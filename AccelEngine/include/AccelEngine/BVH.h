#pragma once
#include <AccelEngine/body.h>
#include <functional>
#include <vector>

namespace AccelEngine
{
    struct BVHNode
    {
        Vector2 minAABB;
        Vector2 maxAABB;

        RigidBody* body;    
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
        real margin = 0.0f; // AABBs closer than this still count as overlapping (speculative contacts)

        BVHTree();
        ~BVHTree();

        void build(const std::vector<RigidBody*>& bodies);
        // Same as build(bodies), spread over `pool`'s threads when given.
        void build(const std::vector<RigidBody*>& bodies, class ThreadPool* pool);
        void destroy();

        void findPairs(std::vector<std::pair<RigidBody*,RigidBody*>>& outPairs);
        // Same result (same order) as findPairs(outPairs), spread over `pool`'s threads when given.
        void findPairs(std::vector<std::pair<RigidBody*,RigidBody*>>& outPairs, class ThreadPool* pool);

        // Calls fn(min, max) for the box of every node (debug drawing, statistics...). The engine itself
        // draws nothing: Sandbox draws the boxes with Renderer2D.
        void forEachNode(const std::function<void(const Vector2 &min, const Vector2 &max)> &fn) const;

    private:
        // Nodes live in one contiguous pool (preorder: parents before children); `root` points into it.
        std::vector<BVHNode> nodes;
        std::vector<char> nodeActive;          // per node: subtree contains an awake, dynamic, collidable body
        std::vector<RigidBody *> lastBodies;   // body set the current topology was built for
        int buildsSinceRebuild = 0;

        struct BuildItem
        {
            Vector2 mn, mx, centroid;
            RigidBody *body;
        };
        std::vector<BuildItem> items;
        std::vector<int> leafNodes;      // node indices of leaves
        std::vector<int> internalNodes;  // node indices of internal nodes, increasing (parents first)
        std::vector<int> leafCount;      // per node: leaves in its subtree
        struct Range { int start, end, node; };
        std::vector<Range> frontier;     // subtrees built in parallel during a rebuild
        std::vector<std::pair<int, int>> heavyTasks, taskWork;
        std::vector<std::vector<std::pair<RigidBody*,RigidBody*>>> chunkPairs;
        std::vector<size_t> chunkOffset;

        void queryPair(int a, int b, std::vector<std::pair<RigidBody*,RigidBody*>>& out,
                       std::vector<std::pair<int, int>>& stack) const;
        void buildTop(int start, int end, int node, int depth);
        void buildRecursive(int start, int end, int node);
        int splitRange(int start, int end, int node);
        void rebuild(const std::vector<RigidBody*>& bodies, class ThreadPool* pool);
        void refresh(class ThreadPool* pool);

        bool AABBOverlap(const Vector2 &minA, const Vector2 &maxA,
                         const Vector2 &minB, const Vector2 &maxB);
    };
}
