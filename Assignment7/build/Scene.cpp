//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <ctime>
#include <cstdlib>

const double epsilon = 0.0001;

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
	Vector3f L_dir, L_indir;
	Intersection p = intersect(ray);
	if( !p.happened) return Vector3f(0, 0, 0);
	if( p.m->hasEmission()) return p.m->getEmission();
	
	
	Intersection x;
	float pdf_light;
	sampleLight( x, pdf_light);
	Ray p_x( p.coords, (x.coords - p.coords).normalized() );
	float p_x_distance_squared = dotProduct( x.coords - p.coords, x.coords - p.coords );
	
	//uint32_t index;
	//Object **hitObject = nullptr;
	if( std::sqrt(p_x_distance_squared) < (intersect(p_x).distance + epsilon ) ){

		Vector3f f_r = p.m->eval( ray.direction, p_x.direction, p.normal );
		L_dir = x.emit * f_r * dotProduct( p_x.direction, p.normal ) * dotProduct( -p_x.direction, x.normal )
			/ p_x_distance_squared / pdf_light ;
	}
	
	//get_random_float()
	//srand(static_cast <unsigned> (time(0)));
	//float random = static_cast<float> (rand() / RAND_MAX );
	if( (get_random_float() > RussianRoulette) ){
		return L_dir + L_indir;
	} else {
		Vector3f wi = p.m->sample(ray.direction, p.normal).normalized();
		Ray r(p.coords, wi);
		Intersection q = intersect(r);
		if(q.happened && (!q.m->hasEmission()) ){
			Vector3f f_r = p.m->eval(ray.direction, wi, p.normal);
			L_indir = castRay(r, depth+1) * f_r * dotProduct(wi, p.normal) 
				/  p.m->pdf(ray.direction, wi, p.normal)  / RussianRoulette; 
		}
	}
	
	return L_dir + L_indir;
		
}