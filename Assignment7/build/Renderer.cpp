//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>

//inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;



// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
	Vector3f eye_pos(278, 273, -800);


	uint32_t quarter_height = scene.height / 4;
	int quarter_size = quarter_height * scene.width;
	
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
	
	std::vector<Vector3f> framebuffer0(quarter_size);
	
	std::vector<Vector3f> framebuffer1(quarter_size);
	
	std::vector<Vector3f> framebuffer2(quarter_size);
	
	std::vector<Vector3f> framebuffer3(quarter_size);

    
    //int m = 0;

    // change the spp value to change sample ammount
    int spp = 64;
    std::cout << "SPP: " << spp << "\n";
	
	std::thread thread0(&Renderer::RenderThread, this, 0, quarter_height, 
		std::ref(scene), std::ref(framebuffer0), spp, eye_pos);
		
	std::thread thread1(&Renderer::RenderThread, this, quarter_height, quarter_height * 2,
		std::ref(scene), std::ref(framebuffer1), spp, eye_pos);
		
	std::thread thread2(&Renderer::RenderThread, this, quarter_height * 2, quarter_height * 3,
		std::ref(scene), std::ref(framebuffer2), spp, eye_pos);
		
	std::thread thread3(&Renderer::RenderThread, this, quarter_height * 3, scene.height, 
		std::ref(scene), std::ref(framebuffer3), spp,  eye_pos);
		
	thread0.join();
	thread1.join();
	thread2.join();
	thread3.join();
	
    UpdateProgress(1.f);
	
	
	for(int i = 0; i < quarter_size; ++i){
		framebuffer[i] = framebuffer0[i];
		framebuffer[i+quarter_size] = framebuffer1[i];
		framebuffer[i+quarter_size * 2] = framebuffer2[i];
		framebuffer[i+quarter_size * 3] = framebuffer3[i];
	}

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

void Renderer::RenderThread(uint32_t bot_height, uint32_t top_height, const Scene& scene, 
	std::vector<Vector3f>& framebuffer, int spp, Vector3f eye_pos)
{	
	int m = 0;
	for (uint32_t j = bot_height; j < top_height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      scene.imageAspectRatio * scene.scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scene.scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
}
	