#include "viewer.h"

#include <filesystem/resolver.h>

void render(Scene* scene, ImageBlock* result, std::string outputName, bool* done)
{
    if(!scene)
        return;

    clock_t t = clock();

    const Camera& camera = *(scene->camera());
    const Integrator& integrator = *(scene->integrator());
    integrator.preprocess(scene);

    float tanfovy2 = tan(camera.fovY()*0.5);    
    int width = camera.vpWidth();
    int height = camera.vpHeight();

    Vector3f camX = camera.right() * tanfovy2 * camera.nearDist() * float { width } / float { height };
    Vector3f camY = -camera.up() * tanfovy2 * camera.nearDist();
    Vector3f camF = camera.direction() * camera.nearDist();

    /// TODO:
    ///  1. iterate over the image pixels
    ///  2. generate a primary ray
    ///  3. call the integartor to compute the color along this ray
    ///  4. write this color in the result image

    for (int pixelx = 0; pixelx < width; pixelx++) {
        for (int pixely = 0; pixely < height; pixely++) {
            
            // Compute relative coordinates of pixel center in the image plane
            float facteurX = -1 + 2 * (pixelx + .5f) / (width - 1);
            float facteurY = -1 + 2 * (pixely + .5f) / (height - 1);

            // Ray starting at camera origin towards the center of the pixel
            Ray ray { camera.position(), (camF + facteurX * camX + facteurY * camY).normalized() };
            ray.recursionLevel = integrator.maxRecursion();
            Color3f pix_color = integrator.Li(scene,ray);
            pix_color.fullClamp();
            result->put(Vector2f ( pixelx, pixely ), pix_color);
        }
    }
        

    t = clock() - t;
    std::cout << "Raytracing time : " << float(t)/CLOCKS_PER_SEC << "s"<<std::endl;

    *done = true;
}

int main(int argc, char *argv[])
{
    getFileResolver()->prepend(DATA_DIR);

    try {
        nanogui::init();
        Viewer *screen = new Viewer();;

        if (argc == 2) {
            /* load file from the command line */
            filesystem::path path(argv[1]);

            if(path.extension() == "scn") { // load scene file
                screen->loadScene(argv[1]);
            }else if(path.extension() == "exr") { // load OpenEXR image
                screen->loadImage(argv[1]);
            }
        }

        /* Enter the application main loop */
        nanogui::mainloop();

        delete screen;
        nanogui::shutdown();
    } catch (const std::exception &e) {
        cerr << "Fatal error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
