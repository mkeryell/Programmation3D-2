
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
    Node& node = m_nodes[nodeId];


    // étape 1 : calculer la boite englobante des faces indexées de m_faces[start] à m_faces[end]
    // (Utiliser la fonction extend de Eigen::AlignedBox3f et la fonction mpMesh->vertexOfFace(int) pour obtenir les coordonnées des sommets des faces)
    Eigen::AlignedBox3f box;
    for (int index = start; index < end; ++index) {
        Eigen::AlignedBox3f face_box;
        for (int pindex = 0; pindex < 3; ++pindex) {
            auto p = m_pMesh->vertexOfFace(index, pindex);
            Eigen::AlignedBox3f point_box { p.position };
            face_box.extend(point_box);
        }
        box.extend(face_box);
    }

    // étape 2 : déterminer si il s'agit d'une feuille (appliquer les critères d'arrêts)

    // Si c'est une feuille, finaliser le noeud et quitter la fonction

    // Si c'est un noeud interne :

    // étape 3 : calculer l'index de la dimension (x=0, y=1, ou z=2) et la valeur du plan de coupe
    // (on découpe au milieu de la boite selon la plus grande dimension)

    // étape 4 : appeler la fonction split pour trier (partiellement) les faces et vérifier si le split a été utile

    // étape 5 : allouer les fils, et les construire en appelant buildNode...
}
