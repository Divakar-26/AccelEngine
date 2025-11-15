#include <AccelEngine/BVH.h>
#include <algorithm>
#include <cmath>
#include <../../Sandbox/include/renderer2D.h> 

using namespace AccelEngine;

BVHTree::BVHTree() : root(nullptr) {}

BVHTree::~BVHTree()
{
    destroy();
}

bool BVHTree::AABBOverlap(const Vector2 &minA, const Vector2 &maxA,
                          const Vector2 &minB, const Vector2 &maxB)
{
    if (maxA.x < minB.x || minA.x > maxB.x) return false;
    if (maxA.y < minB.y || minA.y > maxB.y) return false;
    return true;
}

void BVHTree::destroy()
{
    destroyRecursive(root);
    root = nullptr;
}

void BVHTree::destroyRecursive(BVHNode* node)
{
    if (!node) return;
    destroyRecursive(node->left);
    destroyRecursive(node->right);
    delete node;
}

void BVHTree::build(const std::vector<RigidBody*>& bodies)
{
    destroy();

    if (bodies.empty())
    {
        root = nullptr;
        return;
    }

    std::vector<RigidBody*> temp = bodies;
    root = buildRecursive(temp, 0, temp.size());
}

BVHNode* BVHTree::buildRecursive(std::vector<RigidBody*>& bodies, int start, int end)
{
    int count = end - start;
    if (count <= 0) return nullptr;

    BVHNode* node = new BVHNode();

    if (count == 1)
    {
        node->body = bodies[start];
        node->minAABB = bodies[start]->worldAABBMin;
        node->maxAABB = bodies[start]->worldAABBMax;
        return node;
    }

    Vector2 mn(1e9f, 1e9f);
    Vector2 mx(-1e9f, -1e9f);

    for (int i = start; i < end; i++)
    {
        const Vector2& a = bodies[i]->worldAABBMin;
        const Vector2& b = bodies[i]->worldAABBMax;

        mn.x = std::min(mn.x, a.x);
        mn.y = std::min(mn.y, a.y);
        mx.x = std::max(mx.x, b.x);
        mx.y = std::max(mx.y, b.y);
    }

    node->minAABB = mn;
    node->maxAABB = mx;

    float dx = mx.x - mn.x;
    float dy = mx.y - mn.y;
    int axis = (dx > dy) ? 0 : 1;

    std::sort(bodies.begin() + start, bodies.begin() + end,
              [axis](RigidBody* A, RigidBody* B)
    {
        float ca = (A->worldAABBMin[axis] + A->worldAABBMax[axis]) * 0.5f;
        float cb = (B->worldAABBMin[axis] + B->worldAABBMax[axis]) * 0.5f;
        return ca < cb;
    });

    int mid = start + count / 2;

    node->left = buildRecursive(bodies, start, mid);
    node->right = buildRecursive(bodies, mid, end);

    return node;
}

void BVHTree::findPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs)
{
    outPairs.clear();
    if (!root) return;
    
    queryPairs(root, outPairs);
}

void BVHTree::queryPairs(BVHNode* node, std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs)
{
    if (!node || !node->left || !node->right) return;
    
    queryNodeAgainstTree(node->left, node->right, outPairs);
    
    queryPairs(node->left, outPairs);
    queryPairs(node->right, outPairs);
}

void BVHTree::queryNodeAgainstTree(BVHNode* nodeA, BVHNode* nodeB, 
                                  std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs)
{
    if (!nodeA || !nodeB) return;
    
    if (!AABBOverlap(nodeA->minAABB, nodeA->maxAABB, nodeB->minAABB, nodeB->maxAABB))
        return;
    
    if (nodeA->body && nodeB->body)
    {
        if (nodeA->body->enableCollision && nodeB->body->enableCollision)
            outPairs.push_back({nodeA->body, nodeB->body});
        return;
    }
    
    if (nodeA->body)
    {
        queryNodeAgainstTree(nodeA, nodeB->left, outPairs);
        queryNodeAgainstTree(nodeA, nodeB->right, outPairs);
    }
    else if (nodeB->body)
    {
        queryNodeAgainstTree(nodeA->left, nodeB, outPairs);
        queryNodeAgainstTree(nodeA->right, nodeB, outPairs);
    }
    else
    {
        queryNodeAgainstTree(nodeA->left, nodeB->left, outPairs);
        queryNodeAgainstTree(nodeA->left, nodeB->right, outPairs);
        queryNodeAgainstTree(nodeA->right, nodeB->left, outPairs);
        queryNodeAgainstTree(nodeA->right, nodeB->right, outPairs);
    }
}

// Visualization (same as before)
static void drawNode(BVHNode* node)
{
    if (!node) return;

    SDL_Color col = {255, 255, 0, 255}; // yellow

    Renderer2D::DrawAABBOutline(
        node->minAABB.x,
        node->minAABB.y,
        node->maxAABB.x,
        node->maxAABB.y,
        col
    );

    drawNode(node->left);
    drawNode(node->right);
}

void BVHTree::draw()
{
    drawNode(root);
}