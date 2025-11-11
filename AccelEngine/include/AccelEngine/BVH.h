#pragma once
#include <AccelEngine/body.h>
#include <vector>

namespace AccelEngine
{
    struct BVHNode
    {
        Vector2 minAABB;
        Vector2 maxAABB;

        RigidBody* body;     // leaf if not null
        BVHNode* left;
        BVHNode* right;

        BVHNode()
            : body(nullptr), left(nullptr), right(nullptr),
              minAABB(1e9f, 1e9f), maxAABB(-1e9f, -1e9f) {}
    };

    class BVHTree
    {
    public:
        BVHNode* root;

        BVHTree();
        ~BVHTree();

        void build(const std::vector<RigidBody*>& bodies);
        void destroy();

        void findPairs(std::vector<std::pair<RigidBody*,RigidBody*>>& outPairs);

        // Visualization
        void draw();

    private:
        BVHNode* buildRecursive(std::vector<RigidBody*>& bodies, int start, int end);
        void destroyRecursive(BVHNode* node);
        void queryPairs(BVHNode* node, std::vector<std::pair<RigidBody*,RigidBody*>>& outPairs);
        void queryNodeAgainstTree(BVHNode* nodeA, BVHNode* nodeB, 
                                 std::vector<std::pair<RigidBody*,RigidBody*>>& outPairs);

        bool AABBOverlap(const Vector2 &minA, const Vector2 &maxA,
                         const Vector2 &minB, const Vector2 &maxB);
    };
}