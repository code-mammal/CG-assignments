#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
		
		//if ( objects.size() <= 4) splitMethod = SplitMethod::NAIVE;
		
		switch(splitMethod){
			case SplitMethod::NAIVE: {
				
				Bounds3 centroidBounds;
				for (int i = 0; i < objects.size(); ++i)
					centroidBounds =
						Union(centroidBounds, objects[i]->getBounds().Centroid());
				int dim = centroidBounds.maxExtent();
				switch (dim) {
				case 0:
					std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
						return f1->getBounds().Centroid().x <
							   f2->getBounds().Centroid().x;
					});
					break;
				case 1:
					std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
						return f1->getBounds().Centroid().y <
							   f2->getBounds().Centroid().y;
					});
					break;
				case 2:
					std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
						return f1->getBounds().Centroid().z <
							   f2->getBounds().Centroid().z;
					});
					break;
				}

				auto beginning = objects.begin();
				auto middling = objects.begin() + (objects.size() / 2);
				auto ending = objects.end();
				auto leftshapes = std::vector<Object*>(beginning, middling);
				auto rightshapes = std::vector<Object*>(middling, ending);	

				assert(objects.size() == (leftshapes.size() + rightshapes.size()));

				node->left = recursiveBuild(leftshapes);
				node->right = recursiveBuild(rightshapes);

				node->bounds = Union(node->left->bounds, node->right->bounds);

				break;

			}
			
			case SplitMethod::SAH: {
				Bounds3 centroidBounds;
				for( int i = 0; i < objects.size(); ++i ){
					centroidBounds = Union( centroidBounds, objects[i]->getBounds().Centroid() );	
				}
				int dim = centroidBounds.maxExtent();
				
				const int nBuckets = 12;
				const float SAHTravCost = 0.125f;
				const int SAHInterCost = 1;
				BucketInfo buckets[nBuckets];
				for( int i = 0; i < objects.size(); ++i){
					int b;
					switch(dim){
						case 0: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).x; break;
						case 1: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).y; break;
						case 2: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).z; break;
					}
					if( b == nBuckets) b = nBuckets - 1;
					buckets[b].count++;
					buckets[b].bounds = Union( buckets[b].bounds, objects[i]->getBounds() );
	
				}
				
				float cost[nBuckets - 1];
				for ( int i = 0; i < nBuckets -1; ++i){
					Bounds3 bounds1, bounds2;
					int count1 = 0, count2 = 0;
					for( int j = 0; j < i + 1; ++j){
						count1 += buckets[j].count;
						bounds1 = Union( bounds1, buckets[j].bounds );
					}
					
					for ( int j = i + 1; j < nBuckets; ++j){
						count2 += buckets[j].count;
						bounds2 = Union( bounds2, buckets[j].bounds );
					}
					
					cost[i] = SAHTravCost + ( count1 * bounds1.SurfaceArea() + count2 * bounds2.SurfaceArea() ) 
					/ centroidBounds.SurfaceArea() * SAHInterCost;
				}
				
				float minCost = cost[0];
				int bIndex = 0;
				for ( int i = 0; i < nBuckets -1; ++i){
					if ( cost[i] < minCost ){
						bIndex = i;
						minCost = cost[i];
					}
				}

				auto leftshapes = std::vector<Object*>();
				auto rightshapes = std::vector<Object*>();	
				
				for( int i = 0; i < objects.size(); ++i){
					int b;
					switch(dim){
						case 0: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).x; break;
						case 1: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).y; break;
						case 2: b = nBuckets * centroidBounds.Offset( objects[i]->getBounds().Centroid() ).z; break;
					}
					if( b == nBuckets) b = nBuckets - 1;
					if( b <= bIndex ) leftshapes.push_back( objects[i] );
					else rightshapes.push_back( objects[i] );
	
				}	
				

				assert(objects.size() == (leftshapes.size() + rightshapes.size()));

				node->left = recursiveBuild(leftshapes);
				node->right = recursiveBuild(rightshapes);

				node->bounds = Union(node->left->bounds, node->right->bounds);

				break;
			
			}
			
			
		}
			
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
	Intersection intersect;
	Bounds3 bounds = node->bounds;
	
	std::array<int, 3> dirIsNeg;
	ray.direction.x >=0 ? dirIsNeg[0] = 1 : dirIsNeg[0] = -1;
	ray.direction.y >=0 ? dirIsNeg[1] = 1 : dirIsNeg[1] = -1;
	ray.direction.z >=0 ? dirIsNeg[2] = 1 : dirIsNeg[2] = -1;	
	
	
	
	if( !bounds.IntersectP( ray, ray.direction_inv, dirIsNeg ) )
		return intersect;
	
	if( node->left == nullptr && node->right == nullptr )
		return node->object->getIntersection(ray);
	
	Intersection hitpoint1 = BVHAccel::getIntersection( node->left, ray );
	Intersection hitpoint2 = BVHAccel::getIntersection( node->right, ray );
	
	return ( hitpoint1.distance < hitpoint2.distance) ?  hitpoint1 : hitpoint2 ;

	
}