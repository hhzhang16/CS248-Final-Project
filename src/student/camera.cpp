
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"
#include "../util/rand.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1, Step 2
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    // Y (screen-coord) = 0. This is in a 'view-space' Y of "S_h/2".
    // Because we are given FOV, and because we have a right triangle with angle
    // FOV/2 whose height is "S_h/2", we can solve for S_h/2 by saying:
    //
    // (S_h/2) / 1 = tan(FOV/2), => S_h = 2 * (tan(FOV/2)).

    const float fov_radians = vert_fov * PI_F / (180.0f);
    const float screen_h = 2 * tan(fov_radians / 2); // See derivation above.

    // Screen dimensions, in view space.
    Vec2 screen_wh = Vec2(screen_h * aspect_ratio, screen_h);
    // screen_wh = Vec2(aspect_ratio, 1.0f);

    float view_x = (screen_coord.x * screen_wh.x) - screen_wh.x / 2;
    float view_y = (screen_coord.y * screen_wh.y) - screen_wh.y / 2;

    // Create the point in the view plane.
    Vec3 viewspace_pos = Vec3(view_x, view_y, float(-1.0f));

    if(RNG::coin_flip(0.0005f))
    {
        printf("Screen_HW: (%.2f, %.2f) \t|\t View (X, Y): (%.2f, %.2f) \n",
               screen_wh.x, screen_wh.y,
               view_x, view_y);
        printf("\t FOV (Deg, Rad): (%.2f, %.2f)\n", vert_fov, fov_radians);
    }

    // Shoot a ray from viewspace origin, (0, 0, 0), to the viewspace position
    // for the screen coordinate.
    const Vec3 ray_direction_vs = viewspace_pos - Vec3(0, 0, 0);
    const Vec3 ray_origin_vs = Vec3(0, 0, 0);

    // View Space Coordinates (camera at origin, plane at (X, Y, -1)).
    const Ray camera_ray_vs(ray_origin_vs, ray_direction_vs);

    // World-Space Coordinates. Perform the transformation with iview.
    Ray camera_ray_ws = camera_ray_vs;
    camera_ray_ws.transform(iview);
    
    return camera_ray_ws;
}
