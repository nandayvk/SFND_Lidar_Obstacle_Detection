//
// Created by nandayvk on 7/21/20.
//

#ifndef PLAYBACK_KDTREE_H
#define PLAYBACK_KDTREE_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>

struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct kdtree
{
    Node* root;

    kdtree()
            : root(NULL)
    {}

    void insertHelper(Node*& node, uint depth, std::vector<float> point, int id)
    {
        if(node == NULL)
            node = new Node(point, id);
        else
        {
            uint cd = depth % 3;

            if(point[cd]<((node)->point[cd]))
                insertHelper((node->left), depth+1, point, id);
            else
                insertHelper((node->right), depth+1, point, id);
        }
    }

    void insert(pcl::PointXYZI& cloudpoint, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        std::vector<float> cloudpointVec{cloudpoint.x, cloudpoint.y, cloudpoint.z};
        insertHelper(root, 0, cloudpointVec, id);
    }

    void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, pcl::PointIndices& ids)
    {
        if(node != NULL)
        {
            if( (node->point[0]<=(target[0]+distanceTol) && node->point[0]>=(target[0]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol) && node->point[1]>=(target[1]-distanceTol)) && (node->point[2]<=(target[2]+distanceTol) && node->point[2]>=(target[2]-distanceTol)) )
            {
                float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]) + (node->point[2]-target[2])*(node->point[2]-target[2]));
                if(distance<distanceTol)
                    ids.indices.push_back(node->id);
            }

            if((target[depth%3]-distanceTol)<node->point[depth%3])
                searchHelper(target, node->left, depth+1, distanceTol, ids);
            if((target[depth%3]+distanceTol)>node->point[depth%3])
                searchHelper(target, node->right, depth+1, distanceTol, ids);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    pcl::PointIndices search(pcl::PointXYZI& target, float distanceTol)
    {
        pcl::PointIndices ids;
        std::vector<float> targetVec{target.x, target.y, target.z};
        searchHelper(targetVec, root, 0, distanceTol, ids);

        return ids;
    }


};


#endif //PLAYBACK_KDTREE_H
