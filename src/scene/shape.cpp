
#include "shape.h"
#include "../geometry/util.h"

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

	// Sphere center is at the origin, radius is Sphere::radius
    float radius = this->radius;

    // Calculate the coefficients of the quadratic equation
    Vec3 oc = ray.point; // Ray origin is the same as oc (origin to center)
    float a = dot(ray.dir, ray.dir);
    float b = 2.0f * dot(oc, ray.dir);
    float c = dot(oc, oc) - radius * radius;

    // Solve the quadratic equation
    float discriminant = b * b - 4 * a * c;

    PT::Trace ret;
    ret.origin = ray.point;
    ret.hit = false;

    if (discriminant < 0) {
        // No real roots, the ray does not intersect the sphere
        return ret;
    }

    // Calculate the two roots (t0 and t1)
    float sqrt_discriminant = std::sqrt(discriminant);
    float t0 = (-b - sqrt_discriminant) / (2.0f * a);
    float t1 = (-b + sqrt_discriminant) / (2.0f * a);

    // Ensure t0 is the smaller root
    if (t0 > t1) std::swap(t0, t1);

    // Check if the intersection is within the ray's distance bounds
    if (t0 < ray.dist_bounds.x || t0 > ray.dist_bounds.y) {
        t0 = t1; // Use the second intersection point if the first is out of bounds
        if (t0 < ray.dist_bounds.x || t0 > ray.dist_bounds.y) {
            return ret; // Both intersections are out of bounds
        }
    }

    // Calculate the intersection point
    Vec3 intersection_point = ray.point + t0 * ray.dir;

    // Calculate the normal at the intersection point
    Vec3 normal = intersection_point.unit();

    // Calculate the UV coordinates at the intersection point
    Vec2 uv = Sphere::uv(normal);

    // Fill in the Trace object with intersection details

    ret.origin = ray.point;
    ret.hit = true;       // was there an intersection?
    ret.distance = t0;   // at what distance did the intersection occur?
    ret.position = intersection_point; // where was the intersection?
    ret.normal = normal;   // what was the surface normal at the intersection?
	ret.uv = uv; 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
    return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
