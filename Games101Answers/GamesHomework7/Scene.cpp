//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	Intersection inter = intersect(ray);
	Intersection lightInter;
	if (inter.happened)
	{
		// 如果射线第一次打到光源，直接返回
		if (inter.m->hasEmission())
		{
			if (depth == 0)
			{
				return inter.m->getEmission();
			}
			else return Vector3f(0, 0, 0);
		}

		Vector3f L_dir(0, 0, 0);
		Vector3f L_indir(0, 0, 0);

		// 随机 sample 灯光，用该 sample 的结果判断射线是否击中光源
		
		float pdf_light = 0.0f;
		sampleLight(lightInter, pdf_light);

		// 物体表面法线
		auto& N = inter.normal;
		// 灯光表面法线
		auto& NN = lightInter.normal;

		auto& objPos = inter.coords;
		auto& lightPos = lightInter.coords;

		auto diff = lightPos - objPos;
		auto lightDir = diff.normalized();
		float lightDistance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

		Ray light(objPos, lightDir);
		Intersection light2obj = intersect(light);

		// 如果反射击中光源
		if (light2obj.happened && (light2obj.coords - lightPos).norm() < 1e-2)
		{
			Vector3f f_r = inter.m->eval(ray.direction, lightDir, N);
			L_dir = lightInter.emit * f_r * dotProduct(lightDir, N) * dotProduct(-lightDir, NN) / lightDistance / pdf_light;
		}

		if (get_random_float() < RussianRoulette)
		{
			Vector3f nextDir = inter.m->sample(ray.direction, N).normalized();

			Ray nextRay(objPos, nextDir);
			Intersection nextInter = intersect(nextRay);
			if (nextInter.happened && !nextInter.m->hasEmission())
			{
				float pdf = inter.m->pdf(ray.direction, nextDir, N);
				Vector3f f_r = inter.m->eval(ray.direction, nextDir, N);
				L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextDir, N) / pdf / RussianRoulette;
			}
		}

		return L_dir + L_indir;
	}

	return Vector3f(0, 0, 0);
}
