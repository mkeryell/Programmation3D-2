
#include "bvh.h"
#include "mesh.h"
#include <iostream>

void BVH::build(const Mesh* pMesh, int targetCellSize, int maxDepth)
{
    // store a pointer to the mesh
    m_pMesh = pMesh;
    // allocate the root node
    m_nodes.resize(1);

    if(m_pMesh->nbFaces() <= targetCellSize) { // only one node
        m_nodes[0].box = pMesh->AABB();
        m_nodes[0].first_face_id = 0;
        m_nodes[0].is_leaf = true;
        m_nodes[0].nb_faces = m_pMesh->nbFaces();
        m_faces.resize(m_pMesh->nbFaces());
        for(int i=0; i<m_pMesh->nbFaces(); ++i)
        {
            m_faces[i] = i;
        }
    }else{
        // reserve space for other nodes to avoid multiple memory reallocations
        m_nodes.reserve( std::min<int>(2<<maxDepth, std::log(m_pMesh->nbFaces()/targetCellSize) ) );

        // compute centroids and initialize the face list
        m_centroids.resize(m_pMesh->nbFaces());
        m_faces.resize(m_pMesh->nbFaces());
        for(int i=0; i<m_pMesh->nbFaces(); ++i)
        {
            m_centroids[i] = (m_pMesh->vertexOfFace(i, 0).position + m_pMesh->vertexOfFace(i, 1).position + m_pMesh->vertexOfFace(i, 2).position)/3.f;
            m_faces[i] = i;
        }

        // recursively build the BVH, starting from the root node and the entire list of faces
        buildNode(0, 0, m_pMesh->nbFaces(), 0, targetCellSize, maxDepth);
    }
}

bool BVH::intersect(const Ray& ray, Hit& hit) const
{
    // compute the intersection with the root node
    float tMin, tMax;
    Normal3f n;
    if (::intersect(ray, m_nodes[0].box, tMin, tMax, n))
        return intersectNode(0, ray, hit);
    
    return false;

    // TODO
    // vérifier si on a bien une intersection (en fonction de tMin, tMax, et hit.t()), et si oui appeler intersecNode...en fonction de tMin, tMax, et hit.t()
}

bool BVH::intersectNode(int nodeId, const Ray& ray, Hit& hit) const
{
    const Node& node = m_nodes[nodeId];

    // TODO, deux cas: soit mNodes[nodeId] est une feuille (il faut alors intersecter les triangles du noeud),
    // soit c'est un noeud interne (il faut visiter les fils (ou pas))
    bool has_intersect = false;
    auto setIntersect = [&] (auto&& f) {
        Hit tmp_hit;
        if (f(tmp_hit)) {
            has_intersect = true;
            if (tmp_hit.t() < hit.t()) {
                hit.setT(tmp_hit.t());
                hit.setNormal(tmp_hit.normal());
            }
        }
    };
    float tMin, tMax;
    Normal3f normal;

    if( (!::intersect(ray, node.box, tMin, tMax, normal)) || tMin>hit.t())
        return false;

    if (node.is_leaf) {
        for (int i = 0; i < node.nb_faces; ++i)
            setIntersect([&] (auto& tmp_hit) {
                return m_pMesh->intersectFace(ray, tmp_hit, node.first_face_id + i) && tmp_hit.t() > 0;
            });
    }
    else {
        float tMin1, tMin2, tMax1, tMax2;
        Normal3f normal1, normal2;
        int index1 = node.first_child_id;
        int index2 = node.first_child_id + 1;
        // std::cout << nodeId << " " << index1 << " " << index2 << " " << node.is_leaf << std::endl;
        // std::cout << nodeId << ' ' << (void*) &node << " " << node.first_child_id << std::endl;

        //std::terminate();
        if (::intersect(ray, m_nodes[index1].box, tMin1, tMax1, normal1))
            setIntersect([&] (auto& tmp_hit) { return intersectNode(index1, ray, tmp_hit); });
        if (::intersect(ray, m_nodes[index2].box, tMin2, tMax2, normal2))
            setIntersect([&] (auto& tmp_hit) { return intersectNode(index2, ray, tmp_hit); });
    }
    return has_intersect;
}

/** Sorts the faces with respect to their centroid along the dimension \a dim and spliting value \a split_value.
  * \returns the middle index
  */
int BVH::split(int start, int end, int dim, float split_value)
{
    int l(start), r(end-1);
    while(l<r)
    {
        // find the first on the left
        while(l<end && m_centroids[l](dim) < split_value) ++l;
        while(r>=start && m_centroids[r](dim) >= split_value) --r;
        if(l>r) break;
        std::swap(m_centroids[l], m_centroids[r]);
        std::swap(m_faces[l], m_faces[r]);
        ++l;
        --r;
    }
    return m_centroids[l][dim]<split_value ? l+1 : l;
}

void BVH::buildNode(int nodeId, int start, int end, int level, int targetCellSize, int maxDepth)
{
    // étape 1 : calculer la boite englobante des faces indexées de m_faces[start] à m_faces[end]
    // (Utiliser la fonction extend de Eigen::AlignedBox3f et la fonction mpMesh->vertexOfFace(int) pour obtenir les coordonnées des sommets des faces)
    // Initialize box with first vertex of first face
    Eigen::AlignedBox3f box { m_pMesh->vertexOfFace(start, 0).position };
    for (int index = start; index < end; ++index) {
        for (int pindex = 0; pindex < 3; ++pindex) {
            auto p = m_pMesh->vertexOfFace(index, pindex);
            box.extend(p.position);
        }
    }
    // étape 2 : déterminer si il s'agit d'une feuille (appliquer les critères d'arrêts)

    if (end - start <= targetCellSize || level >= maxDepth) {
    // Si c'est une feuille, finaliser le noeud et quitter la fonction
        m_nodes[nodeId].is_leaf = true;
        m_nodes[nodeId].nb_faces = end - start;
        m_nodes[nodeId].first_face_id = start;
        m_nodes[nodeId].box = box;
        return;
    }


    // Si c'est un noeud interne :
    // étape 3 : calculer l'index de la dimension (x=0, y=1, ou z=2) et la valeur du plan de coupe
    // (on découpe au milieu de la boite selon la plus grande dimension)
    const auto max_corner = box.max();
    const auto min_corner = box.min();

    auto x_diff = max_corner.x() - min_corner.x();
    auto y_diff = max_corner.y() - min_corner.y();
    auto z_diff = max_corner.z() - min_corner.z();

    int dim;
    if (x_diff > y_diff && x_diff > z_diff)
        dim = 0;
    else if (y_diff > z_diff)
        dim = 1;
    else 
        dim = 2;
    auto split_value = (max_corner(dim) + min_corner(dim)) / 2;

    // étape 4 : appeler la fonction split pour trier (partiellement) les faces et vérifier si le split a été utile
    auto mid_id = split(start, end, dim, split_value);

    // étape 5 : allouer les fils, et les construire en appelant buildNode...
    m_nodes.push_back({});
    m_nodes.push_back({});

    m_nodes[nodeId].first_child_id = m_nodes.size() - 2;
    m_nodes[nodeId].box = box;
    m_nodes[nodeId].nb_faces = end - start;

    buildNode(m_nodes[nodeId].first_child_id, start, mid_id, level+1, targetCellSize, maxDepth);
    buildNode(m_nodes[nodeId].first_child_id + 1, mid_id, end, level+1, targetCellSize, maxDepth);
}
