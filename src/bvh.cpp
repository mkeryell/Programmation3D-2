
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
    if (!::intersect(ray, m_nodes[0].box, tMin, tMax, n))
        return false;

    return intersectNode(0, ray, hit);
    

    // TODO
    // vérifier si on a bien une intersection (en fonction de tMin, tMax, et hit.t()), et si oui appeler intersecNode...en fonction de tMin, tMax, et hit.t()
}

bool BVH::intersectNode(int nodeId, const Ray& ray, Hit& hit) const
{


    // TODO, deux cas: soit mNodes[nodeId] est une feuille (il faut alors intersecter les triangles du noeud),
    // soit c'est un noeud interne (il faut visiter les fils (ou pas))
    bool has_intersect = false;
    const auto& node = m_nodes[nodeId];

    float tMin, tMax;
    Normal3f normal;

    if( (!::intersect(ray, node.box, tMin, tMax, normal)) || tMin>hit.t())
        return false;

    if (node.is_leaf) {
        for (int i = 0; i < node.nb_faces; ++i) {
            Hit tmp_hit;
            if (m_pMesh->intersectFace(ray, tmp_hit, node.first_face_id + i)) {
                has_intersect = true;
                if (tmp_hit.t() < hit.t()) {
                    hit.setT(tmp_hit.t());
                    hit.setNormal(tmp_hit.normal());
                }
            }
        }
    }
    else {
        for (int i = 0; i < node.nb_faces; ++i) {
            Hit tmp_hit;
            if (intersectNode(node.first_child_id + i, ray, tmp_hit)) {
                has_intersect = true;
                if (tmp_hit.t() < hit.t()) {
                    hit.setT(tmp_hit.t());
                    hit.setNormal(tmp_hit.normal());
                }
            }
        }
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
    auto &node = m_nodes[nodeId];


    // étape 1 : calculer la boite englobante des faces indexées de m_faces[start] à m_faces[end]
    // (Utiliser la fonction extend de Eigen::AlignedBox3f et la fonction mpMesh->vertexOfFace(int) pour obtenir les coordonnées des sommets des faces)
    Eigen::AlignedBox3f box;
    for (int index = start; index < end; ++index) {
        int pindex = 0;
        if (index == start) {
            auto p1 = m_pMesh->vertexOfFace(index, 0);
            Eigen::AlignedBox3f box { p1.position };
            pindex = 1;
        }
        for (; pindex < 3; ++pindex) {
            auto p = m_pMesh->vertexOfFace(index, pindex);
            box.extend(p.position);
        }
    }
    // étape 2 : déterminer si il s'agit d'une feuille (appliquer les critères d'arrêts)

    if (end - start <= targetCellSize || level >= maxDepth) {
    // Si c'est une feuille, finaliser le noeud et quitter la fonction
        node.is_leaf = true;
        node.nb_faces = start - end;
        node.first_face_id = start;
        node.box = box;
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
    auto mid_id = split(start, end, dim, split_value); //Utile???

    // étape 5 : allouer les fils, et les construire en appelant buildNode...
    Node firstChild {};
    //firstChild.first_face_id = start;
    //firstChild.nb_faces = mid_id - start;

    Node secondChild {};
    //secondChild.first_face_id = mid_id;
    //secondChild.nb_faces = end - mid_id;

    m_nodes.push_back(firstChild);
    m_nodes.push_back(secondChild);
    
    node.first_child_id = m_nodes.size() - 2;
    node.box = box;
    node.nb_faces = 2;

    buildNode(m_nodes.size() - 2, start, mid_id, level+1, targetCellSize, maxDepth);
    buildNode(m_nodes.size() - 1, mid_id, end, level+1, targetCellSize, maxDepth);
}
