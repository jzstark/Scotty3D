
#include "samplers.h"
#include "../util/rand.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling

    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()
	float x = rng.unit() * size.x;
    float y = rng.unit() * size.y;
    return Vec2(x, y);
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec2 Circle::sample(RNG &rng) const {
	//A3EC - bokeh - circle sampling

    // Return a point selected uniformly at random from a circle defined by its
	// center and radius.
    // Useful function: rng.unit()

    return Vec2{};
}

float Circle::pdf(Vec2 at) const {
	//A3EC - bokeh - circle pdf

	// Return the pdf of sampling the point 'at' for a circle defined by its
	// center and radius.

    return 1.f;
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
	// Generate a random azimuthal angle phi in the range [0, 2*pi)
    
	/* float phi = rng.unit() * 2.0f * PI_F;

    float cos_theta = 1.0f - 2.0f * rng.unit();
    float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

    float x = sin_theta * std::cos(phi);
    float y = sin_theta * std::sin(phi);
    float z = cos_theta;

    return Vec3(x, y, z); */

	float Xi1 = rng.unit();
    if(rng.coin_flip(0.5f))
        Xi1 = -1 * Xi1;
    float Xi2 = rng.unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    theta = std::clamp(theta, 0.0f, PI_F);
    phi = std::clamp(phi, 0.0f, 2.0f * PI_F);

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    for(size_t j = 0; j < h; j++)
    {
        for(size_t i = 0; i < w; i++)
        {
            float theta = PI_F - ((float)j/(float)h)*PI_F;
            theta = std::clamp(theta, 0.0f, PI_F);
            float pdf = image.at(i,j).luma()*sin(theta);
            total += pdf;
            _pdf.push_back(pdf);
            _cdf.push_back(total);
        }
    }
    for(auto& p : _pdf)
        p *= 1.0f / total;
    for(auto& c : _cdf)
        c *= 1.0f / total;
}


//TODO: WARNING - does not pass the tests; it could also be because of the A1T6 implementation of bi-/tri-linear sampler
Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
		Sphere::Uniform uniform_sampler;
		return uniform_sampler.sample(rng);
	}
	// Step 2: Importance sampling
	// Use your importance sampling data structure to generate a sample direction.
	// Tip: std::upper_bound
	float p = rng.unit();
    auto i = std::upper_bound(_cdf.begin(), _cdf.end(), p);

    size_t index = i - _cdf.begin();
    float theta = PI_F - PI_F * (index / w)/(float)h;
    float phi = 2.0f * PI_F* (index % w)/(float)w;
    theta = std::clamp(theta, 0.0f, PI_F);
    phi = std::clamp(phi, 0.0f, 2.0f * PI_F);

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);

}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		Sphere::Uniform uniform_sampler;
		return uniform_sampler.pdf(dir);
	} 
	// A3T7 - image sampler importance sampling pdf
	// What is the PDF of this distribution at a particular direction?
	
	float phi = atan2(dir.z, dir.x);
    if(phi < 0)
        phi += 2.0f * PI_F;
    
    float theta = acosf(dir.y);

    size_t phi_i = (size_t)std::round((w-1) * phi/(2.0f * PI_F));
    size_t theta_i = (size_t)std::round((h-1) * (PI_F - theta)/PI_F);
    size_t index = theta_i * w + phi_i;
    float Jacobian = w * h / (2.0f * PI_F * PI_F * sinf(theta));
    return _pdf[index] * Jacobian;

}

} // namespace Samplers
