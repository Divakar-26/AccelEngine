#include <AccelEngine/BVH.h>
#include <AccelEngine/thread_pool.h>
#include <algorithm>
#include <cmath>
#include <cstring>

using namespace AccelEngine;

namespace
{
    // Rebuild the topology every N builds; in between only the boxes are refitted (O(n), no sorting).
    constexpr int kRebuildInterval = 8;

    inline bool isActive(const RigidBody *b)
    {
        return b->enableCollision && b->isAwake && !b->isStatic();
    }
}

BVHTree::BVHTree() : root(nullptr) {}

BVHTree::~BVHTree()
{
    destroy();
}

bool BVHTree::AABBOverlap(const Vector2 &minA, const Vector2 &maxA,
                          const Vector2 &minB, const Vector2 &maxB)
{
    if (maxA.x + margin < minB.x || minA.x > maxB.x + margin) return false;
    if (maxA.y + margin < minB.y || minA.y > maxB.y + margin) return false;
    return true;
}

void BVHTree::destroy()
{
    nodes.clear();
    nodeActive.clear();
    lastBodies.clear();
    leafNodes.clear();
    internalNodes.clear();
    leafCount.clear();
    root = nullptr;
}

namespace
{
    template <typename F>
    void runParallel(ThreadPool *pool, int count, int grain, F &&fn)
    {
        if (pool)
            pool->parallelFor(count, grain, fn);
        else
        {
            const int g = std::max(grain, 1);
            for (int c = 0, lo = 0; lo < count; c++, lo += g)
                fn(lo, std::min(count, lo + g), c);
        }
    }
}

void BVHTree::build(const std::vector<RigidBody*>& bodies)
{
    build(bodies, nullptr);
}

void BVHTree::build(const std::vector<RigidBody*>& bodies, ThreadPool* pool)
{
    if (bodies.empty())
    {
        destroy();
        return;
    }

    const bool sameSet = !nodes.empty() && bodies.size() == lastBodies.size() &&
                         std::memcmp(bodies.data(), lastBodies.data(), bodies.size() * sizeof(RigidBody *)) == 0;

    if (!sameSet || ++buildsSinceRebuild >= kRebuildInterval)
        rebuild(bodies, pool);
    refresh(pool);
}

// Leaves: copy the body's box and awake state (parallel; this is where the scattered body memory is read).
// Internal nodes: merge children, parents after children (serial, but only touches the contiguous node array).
void BVHTree::refresh(ThreadPool* pool)
{
    nodeActive.resize(nodes.size());
    runParallel(pool, (int)leafNodes.size(), 1024, [this](int lo, int hi, int)
    {
        for (int k = lo; k < hi; k++)
        {
            BVHNode &n = nodes[leafNodes[k]];
            n.minAABB = n.body->worldAABBMin;
            n.maxAABB = n.body->worldAABBMax;
            nodeActive[leafNodes[k]] = isActive(n.body);
        }
    });
    BVHNode *base = nodes.data();
    for (size_t k = internalNodes.size(); k-- > 0;)
    {
        const int i = internalNodes[k];
        BVHNode &n = base[i];
        const BVHNode &l = *n.left, &r = *n.right;
        n.minAABB = Vector2(std::min(l.minAABB.x, r.minAABB.x), std::min(l.minAABB.y, r.minAABB.y));
        n.maxAABB = Vector2(std::max(l.maxAABB.x, r.maxAABB.x), std::max(l.maxAABB.y, r.maxAABB.y));
        nodeActive[i] = nodeActive[n.left - base] | nodeActive[n.right - base];
    }
}

void BVHTree::rebuild(const std::vector<RigidBody*>& bodies, ThreadPool* pool)
{
    buildsSinceRebuild = 0;
    lastBodies = bodies;
    const int n = (int)bodies.size();

    items.resize(n);
    runParallel(pool, n, 2048, [&](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
        {
            RigidBody *b = bodies[i];
            items[i] = {b->worldAABBMin, b->worldAABBMax, (b->worldAABBMin + b->worldAABBMax) * 0.5f, b};
        }
    });

    // Preorder layout: the subtree of a range with k leaves occupies exactly 2k-1 consecutive nodes, so
    // every node's index is known up front and subtrees can be built independently (in parallel).
    nodes.assign(2 * n - 1, BVHNode());
    root = &nodes[0];

    frontier.clear();
    const int depthLimit = pool ? 6 : 0; // up to 64 independent subtrees
    buildTop(0, n, 0, depthLimit);
    runParallel(pool, (int)frontier.size(), 1, [this](int lo, int hi, int)
    {
        for (int f = lo; f < hi; f++)
            buildRecursive(frontier[f].start, frontier[f].end, frontier[f].node);
    });

    // topology bookkeeping (only changes on rebuild)
    leafNodes.clear();
    internalNodes.clear();
    leafCount.assign(nodes.size(), 0);
    for (int i = 0; i < (int)nodes.size(); i++)
        (nodes[i].body ? leafNodes : internalNodes).push_back(i);
    for (size_t k = nodes.size(); k-- > 0;)
        leafCount[k] = nodes[k].body ? 1 : leafCount[nodes[k].left - nodes.data()] + leafCount[nodes[k].right - nodes.data()];
}

// Splits [start, end) at the median centroid of the longest axis; returns mid. Sets the node's box.
int BVHTree::splitRange(int start, int end, int node)
{
    BVHNode &n = nodes[node];
    Vector2 mn(1e30f, 1e30f), mx(-1e30f, -1e30f);
    Vector2 cmn(1e30f, 1e30f), cmx(-1e30f, -1e30f);
    for (int i = start; i < end; i++)
    {
        const BuildItem &it = items[i];
        mn.x = std::min(mn.x, it.mn.x); mn.y = std::min(mn.y, it.mn.y);
        mx.x = std::max(mx.x, it.mx.x); mx.y = std::max(mx.y, it.mx.y);
        cmn.x = std::min(cmn.x, it.centroid.x); cmn.y = std::min(cmn.y, it.centroid.y);
        cmx.x = std::max(cmx.x, it.centroid.x); cmx.y = std::max(cmx.y, it.centroid.y);
    }
    n.minAABB = mn;
    n.maxAABB = mx;

    const int axis = (cmx.x - cmn.x) > (cmx.y - cmn.y) ? 0 : 1;
    const int mid = start + (end - start) / 2;
    std::nth_element(items.begin() + start, items.begin() + mid, items.begin() + end,
                     [axis](const BuildItem &a, const BuildItem &b)
                     { return a.centroid[axis] < b.centroid[axis]; });
    n.left = &nodes[node + 1];
    n.right = &nodes[node + 2 * (mid - start)]; // after the left subtree's 2*(mid-start)-1 nodes
    return mid;
}

void BVHTree::buildTop(int start, int end, int node, int depth)
{
    if (depth == 0 || end - start < 512)
    {
        frontier.push_back({start, end, node});
        return;
    }
    const int mid = splitRange(start, end, node);
    buildTop(start, mid, node + 1, depth - 1);
    buildTop(mid, end, node + 2 * (mid - start), depth - 1);
}

void BVHTree::buildRecursive(int start, int end, int node)
{
    if (end - start == 1)
    {
        BVHNode &n = nodes[node];
        n.body = items[start].body;
        n.minAABB = items[start].mn;
        n.maxAABB = items[start].mx;
        return;
    }
    const int mid = splitRange(start, end, node);
    buildRecursive(start, mid, node + 1);
    buildRecursive(mid, end, node + 2 * (mid - start));
}

void BVHTree::queryPair(int startA, int startB, std::vector<std::pair<RigidBody*, RigidBody*>>& out,
                        std::vector<std::pair<int, int>>& stack) const
{
    const BVHNode *base = nodes.data();
    auto index = [base](const BVHNode *n) { return (int)(n - base); };
    auto overlap = [this](const BVHNode &a, const BVHNode &b)
    {
        return !(a.maxAABB.x + margin < b.minAABB.x || a.minAABB.x > b.maxAABB.x + margin ||
                 a.maxAABB.y + margin < b.minAABB.y || a.minAABB.y > b.maxAABB.y + margin);
    };

    stack.clear();
    stack.push_back({startA, startB});
    while (!stack.empty())
    {
        auto [ai, bi] = stack.back();
        stack.pop_back();
        if (!nodeActive[ai] && !nodeActive[bi])
            continue; // nothing awake on either side
        const BVHNode &a = base[ai];
        const BVHNode &b = base[bi];
        if (!overlap(a, b))
            continue;

        if (a.body && b.body)
        {
            if (a.body->enableCollision && b.body->enableCollision && RigidBody::shouldCollide(*a.body, *b.body))
                out.push_back({a.body, b.body});
        }
        else if (b.body || (!a.body && (a.maxAABB.x - a.minAABB.x) + (a.maxAABB.y - a.minAABB.y) >
                                            (b.maxAABB.x - b.minAABB.x) + (b.maxAABB.y - b.minAABB.y)))
        {
            // descend the larger (or the only internal) node
            stack.push_back({index(a.left), bi});
            stack.push_back({index(a.right), bi});
        }
        else
        {
            stack.push_back({ai, index(b.left)});
            stack.push_back({ai, index(b.right)});
        }
    }
}

void BVHTree::findPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs)
{
    findPairs(outPairs, nullptr);
}

void BVHTree::findPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs, ThreadPool* pool)
{
    outPairs.clear();
    if (!root)
        return;

    // Every pair of overlapping leaves meets at exactly one internal node (their lowest common ancestor),
    // so testing each active internal node's two children against each other finds every pair once.
    // Nodes with big subtrees (near the root) are split into smaller child-pair tasks first so that no
    // single task dominates; everything else is processed straight from the node list in parallel.
    const BVHNode *base = nodes.data();
    auto overlap = [this](const BVHNode &a, const BVHNode &b)
    {
        return !(a.maxAABB.x + margin < b.minAABB.x || a.minAABB.x > b.maxAABB.x + margin ||
                 a.maxAABB.y + margin < b.minAABB.y || a.minAABB.y > b.maxAABB.y + margin);
    };
    const int heavy = std::max(256, (int)(nodes.size() / 64));

    heavyTasks.clear();
    taskWork.clear();
    for (int i : internalNodes)
        if (leafCount[i] > heavy && nodeActive[i])
            taskWork.push_back({(int)(base[i].left - base), (int)(base[i].right - base)});
    while (!taskWork.empty())
    {
        auto [a, b] = taskWork.back();
        taskWork.pop_back();
        if ((!nodeActive[a] && !nodeActive[b]) || !overlap(base[a], base[b]))
            continue;
        const BVHNode &na = base[a], &nb = base[b];
        if (leafCount[a] + leafCount[b] <= heavy || (na.body && nb.body))
        {
            heavyTasks.push_back({a, b});
            continue;
        }
        if (!na.body && (nb.body || leafCount[a] >= leafCount[b]))
        {
            taskWork.push_back({(int)(na.left - base), b});
            taskWork.push_back({(int)(na.right - base), b});
        }
        else
        {
            taskWork.push_back({a, (int)(nb.left - base)});
            taskWork.push_back({a, (int)(nb.right - base)});
        }
    }

    const int lightCount = (int)internalNodes.size();
    const int lightGrain = 128, heavyGrain = 8;
    const int lightChunks = (lightCount + lightGrain - 1) / lightGrain;
    const int heavyChunks = ((int)heavyTasks.size() + heavyGrain - 1) / heavyGrain;
    const int chunks = lightChunks + heavyChunks;
    if ((int)chunkPairs.size() < chunks)
        chunkPairs.resize(chunks);

    runParallel(pool, lightCount, lightGrain, [&](int lo, int hi, int chunk)
    {
        thread_local std::vector<std::pair<int, int>> stack;
        auto &out = chunkPairs[chunk];
        out.clear();
        for (int t = lo; t < hi; t++)
        {
            const int i = internalNodes[t];
            if (leafCount[i] > heavy || !nodeActive[i])
                continue;
            queryPair((int)(base[i].left - base), (int)(base[i].right - base), out, stack);
        }
    });
    runParallel(pool, (int)heavyTasks.size(), heavyGrain, [&](int lo, int hi, int chunk)
    {
        thread_local std::vector<std::pair<int, int>> stack;
        auto &out = chunkPairs[lightChunks + chunk];
        out.clear();
        for (int t = lo; t < hi; t++)
            queryPair(heavyTasks[t].first, heavyTasks[t].second, out, stack);
    });

    // concatenate in chunk order (deterministic regardless of thread timing)
    chunkOffset.resize(chunks + 1);
    chunkOffset[0] = 0;
    for (int c = 0; c < chunks; c++)
        chunkOffset[c + 1] = chunkOffset[c] + chunkPairs[c].size();
    outPairs.resize(chunkOffset[chunks]);
    auto copy = [&](int lo, int hi, int)
    {
        for (int c = lo; c < hi; c++)
            std::copy(chunkPairs[c].begin(), chunkPairs[c].end(), outPairs.begin() + chunkOffset[c]);
    };
    if (pool)
        pool->parallelFor(chunks, 16, copy);
    else
        copy(0, chunks, 0);
}

void BVHTree::forEachNode(const std::function<void(const Vector2 &, const Vector2 &)> &fn) const
{
    for (const BVHNode &n : nodes)
        fn(n.minAABB, n.maxAABB);
}
