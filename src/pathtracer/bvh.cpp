
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>

#include <algorithm>
#include <limits>
#include <vector>

#include <iostream>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh

	// Keep these
    nodes.clear();
    primitives = std::move(prims);

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	//TODO
	root_idx = build_recursive(0, primitives.size(), max_leaf_size);
	return;
}


template <typename Primitive>
size_t BVH<Primitive>::build_recursive(size_t start, size_t end, size_t max_leaf_size) {
    // Compute the bounding box for the current set of primitives
    BBox bbox;
    for (size_t i = start; i < end; ++i) {
        bbox.enclose(primitives[i].bbox());
    }

    // Create a leaf node if the number of primitives is below the threshold
    if (end - start <= max_leaf_size) {
        return new_node(bbox, start, end - start, 0, 0); 
    }

    // Initialize variables for the best split
    size_t best_axis = 0;
    size_t best_bucket = 0;
    float best_cost = std::numeric_limits<float>::infinity();

    // Number of buckets for partitioning
    const size_t num_buckets = 12;
	
	std::vector<Vec3> c = bbox.corners();
	Vec3 allmin = c[0];
	Vec3 allmax = c[7];

    // Iterate over axes x, y, z
    for (size_t axis = 0; axis < 3; ++axis) {
        // Initialize buckets
        std::vector<SAHBucketData> buckets(num_buckets);

        // Assign primitives to buckets 
        for (size_t i = start; i < end; ++i) {
            Vec3 centroid = primitives[i].bbox().center();
			float ratio = (centroid[axis] - allmin[axis]) / (allmax[axis] - allmin[axis]);
            size_t bucket_index = std::min(static_cast<size_t>(num_buckets * ratio), num_buckets - 1);

            buckets[bucket_index].bb.enclose(primitives[i].bbox());
            buckets[bucket_index].num_prims++;
        }

        // Evaluate partitions using SAH
        for (size_t i = 0; i < num_buckets - 1; ++i) {
            BBox bbox_left, bbox_right;
            size_t count_left = 0, count_right = 0;

            for (size_t j = 0; j <= i; ++j) {
                bbox_left.enclose(buckets[j].bb);
                count_left += buckets[j].num_prims;
            }
            for (size_t j = i + 1; j < num_buckets; ++j) {
                bbox_right.enclose(buckets[j].bb);
                count_right += buckets[j].num_prims;
            }

            float cost = (count_left * bbox_left.surface_area() + count_right * bbox_right.surface_area()) / bbox.surface_area();
            if (cost < best_cost) {
                best_cost = cost;
                best_axis = axis;
                best_bucket = i;
            }
        }
    }

    // Partition primitives based on the best split
    auto mid = std::partition(primitives.begin() + start, primitives.begin() + end, [&](const Primitive& prim) {
        Vec3 centroid = prim.bbox().center();
		float ratio = (centroid[best_axis] - allmin[best_axis]) / (allmax[best_axis] - allmin[best_axis]);
        size_t bucket_index = std::min(static_cast<size_t>(num_buckets * ratio), num_buckets - 1);
        return bucket_index <= best_bucket;
    });

    size_t mid_index = std::distance(primitives.begin(), mid);

    // Recursively build the left and right child nodes
	size_t left_child = build_recursive(start, mid_index, max_leaf_size);
    size_t right_child = build_recursive(mid_index, end, max_leaf_size);

    // Create and return the internal node
    return new_node(bbox, start, end - start, left_child, right_child);
}


template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
