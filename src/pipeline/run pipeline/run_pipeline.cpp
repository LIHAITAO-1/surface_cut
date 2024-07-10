
#include "run_pipeline.h"
#include "TriIntersectionTri.h"
#include <vector>
#include <list>
#include <array>

using namespace std;
using namespace base_type;

enum Vtx_state {
	Inside,
	OnEdge,
	OnVtx,
	Other
};

enum Tri_tri_cut_case {
	P1_P2_Inside = 1,

	P1_Inside_P2_OnEdge,
	P1_OnEdge_P2_Inside,

	P1_P2_OnEdge,

	P1_OnVtx_P2_Inside,
	P1_Inside_P2_OnVtx,

	P1_OnVtx_P2_OnEdge,
	P1_OnEdge_P2_OnVtx,

	P1_P2_OnVtx,
	No_Cut
};

struct Cut_result {
	Tri_tri_cut_case cut_case;
	Vertex* c_v[2];
	Edge* c_e[2];
};

struct AABB {
	Vector3 min;
	Vector3 max;

	AABB() : min{ FLT_MAX, FLT_MAX, FLT_MAX }, max{ -FLT_MAX, -FLT_MAX, -FLT_MAX } {}

	void expand(const Vector3& point) {
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);
		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);
	}

	bool overlaps(const AABB& other) const {
		return (min.x <= other.max.x && max.x >= other.min.x &&
			min.y <= other.max.y && max.y >= other.min.y &&
			min.z <= other.max.z && max.z >= other.min.z);
	}
};

AABB computeAABB(const Face& face) {
	AABB box;
	box.expand(face.p1->position);
	box.expand(face.p2->position);
	box.expand(face.p3->position);
	return box;
}

bool isIntersecting(const AABB& box1, const AABB& box2) {
	return (box1.min.x <= box2.max.x && box1.max.x >= box2.min.x &&
		box1.min.y <= box2.max.y && box1.max.y >= box2.min.y &&
		box1.min.z <= box2.max.z && box1.max.z >= box2.min.z);
}

struct Object {
	AABB bounds;
	Face* face;
	// 其他物体相关的属性
};

struct BVHNode {
	AABB bounds;
	BVHNode* left;
	BVHNode* right;
	Object* object;

	BVHNode() : left(nullptr), right(nullptr), object(nullptr) {}
};

BVHNode* buildBVH(std::vector<Object*>& objects) {
	if (objects.empty()) return nullptr;

	BVHNode* node = new BVHNode();
	AABB bounds;
	for (const auto& obj : objects) {
		bounds.expand(obj->bounds.min);
		bounds.expand(obj->bounds.max);
	}
	node->bounds = bounds;

	if (objects.size() == 1) {
		node->object = objects[0];
		return node;
	}

	// 分割轴选择和排序
	int axis = 0; // 0: x, 1: y, 2: z
	if (bounds.max.y - bounds.min.y > bounds.max.x - bounds.min.x) axis = 1;
	if (bounds.max.z - bounds.min.z > bounds.max.x - bounds.min.x && bounds.max.z - bounds.min.z > bounds.max.y - bounds.min.y) axis = 2;

	std::sort(objects.begin(), objects.end(), [axis](Object* a, Object* b) {
		if (axis == 0)
			return a->bounds.min.x < b->bounds.min.x;
		if (axis == 1)
			return a->bounds.min.y < b->bounds.min.y;
		if (axis == 2)
			return a->bounds.min.z < b->bounds.min.z;
		});

	size_t mid = objects.size() / 2;
	std::vector<Object*> left(objects.begin(), objects.begin() + mid);
	std::vector<Object*> right(objects.begin() + mid, objects.end());

	node->left = buildBVH(left);
	node->right = buildBVH(right);

	return node;
}
void insertObject(BVHNode*& node, Object* object) {
	if (!node) {
		node = new BVHNode();
		node->object = object;
		node->bounds = object->bounds;
		return;
	}

	if (node->object) {
		Object* existingObject = node->object;
		node->object = nullptr;
		node->left = new BVHNode();
		node->left->object = existingObject;
		node->left->bounds = existingObject->bounds;

		node->right = new BVHNode();
		node->right->object = object;
		node->right->bounds = object->bounds;
	}
	else {
		AABB leftBounds = node->left ? node->left->bounds : AABB();
		AABB rightBounds = node->right ? node->right->bounds : AABB();

		leftBounds.expand(object->bounds.min);
		leftBounds.expand(object->bounds.max);

		rightBounds.expand(object->bounds.min);
		rightBounds.expand(object->bounds.max);

		if (leftBounds.max.x - leftBounds.min.x <
			rightBounds.max.x - rightBounds.min.x) {
			insertObject(node->left, object);
			node->bounds.expand(node->left->bounds.min);
			node->bounds.expand(node->left->bounds.max);
		}
		else {
			insertObject(node->right, object);
			node->bounds.expand(node->right->bounds.min);
			node->bounds.expand(node->right->bounds.max);
		}
	}
}
bool removeObject(BVHNode*& node, Object* object) {
	if (!node) return false;

	if (node->object == object) {
		delete node;
		node = nullptr;
		return true;
	}

	bool removed = false;
	if (node->left && removeObject(node->left, object)) {
		removed = true;
		if (!node->left && !node->right) {
			delete node;
			node = nullptr;
		}
	}
	else if (node->right && removeObject(node->right, object)) {
		removed = true;
		if (!node->left && !node->right) {
			delete node;
			node = nullptr;
		}
	}

	return removed;
}

bool tri_tri_cut(base_type::Face* f1, base_type::Face* f2, base_type::Vector3& p1, base_type::Vector3& p2){
	if (!isIntersecting(computeAABB(*f1), computeAABB(*f2))) {
		return false;
	}
	Triangle tri1(f1->p1->position, f1->p2->position, f1->p3->position);
	Triangle tri2(f2->p1->position, f2->p2->position, f2->p3->position);

	vector<Vector3> pts;
	if (ComputeLineWithTwoTriangle(tri1, tri2, pts)) {
		p1 = pts[0];
		p2 = pts[1];
		return true;
	}
	else
		return false;
};

bool inFaceVector(Face* f, vector<Face*> FaceArray) {
	return std::find(FaceArray.begin(), FaceArray.end(), f) != FaceArray.end();
}

void AABB_intersection(BVHNode* root, base_type::Face* f_insert, vector<Face*>& interFaceArray, vector<Face*>& interFaceDeleteArray, bool& b) {
	if (root == nullptr)
		return;

	if (!isIntersecting(computeAABB(*f_insert), root->bounds))
		return;

	if (root->left == nullptr && root->right == nullptr) {
		Vector3 p1, p2;
		if (tri_tri_cut(root->object->face, f_insert, p1, p2)) {
			if (!inFaceVector(root->object->face, interFaceDeleteArray))
				interFaceArray.push_back(root->object->face);
			else
				b = true;
		}
	}
	AABB_intersection(root->left, f_insert, interFaceArray, interFaceDeleteArray, b);
	AABB_intersection(root->right, f_insert, interFaceArray, interFaceDeleteArray, b);
};

Vtx_state get_vtx_state(double a, double b) {
	if ((a + b) < 1 - 1e-6 && a > 1e-6 && b > 1e-6) {
		return Inside;
	}
	if ((abs(a) < 1e-6 && abs(b) < 1e-6) ||
		(abs(abs(a) - 1) < 1e-6 && abs(b) < 1e-6) ||
		(abs(abs(b) - 1) < 1e-6 && abs(a) < 1e-6)) {
		return OnVtx;
	}
	if (abs(a) < 1e-6 || abs(b) < 1e-6 || abs((a + b) - 1) < 1e-6) {
		return OnEdge;
	}
	return Other;
	};

std::pair<double, double> point_uv_calulate_triangle(Triangle3d tri, base_type::Vector3 intersection) {
	auto u = tri.p2 - tri.p1;
	auto v = tri.p3 - tri.p1;
	auto Q = tri.p1;

	base_type::Vector3 cvu = cross(u, v);

	base_type::Vector3 planar_hitpt_vector = intersection - Q;
	auto w = cvu / dot(cvu, cvu);
	auto alpha = dot(w, cross(planar_hitpt_vector, v));
	auto beta = dot(w, cross(u, planar_hitpt_vector));

	return { alpha, beta };
	};

std::pair<Vertex*, std::array<Edge*, 2>> edge_split(Triangle_Soup_Mesh& mesh, base_type::Edge* e, const base_type::Vector3& p, Face*& new_f1, Face*& new_f2, vector<Face*>& interFaceArray, vector<Face*>& interFaceDeleteArray, vector<Face*>& interFaceAddArray){

	//     orig
	//     /|\            /|\
        //    / | \          / | \
        //   /f1|f2\        /  |  \
        //  /   |   \  ->  /___|___\
        //  \   |   /      \   |   /
		//   \  |  /        \  |  /
		//    \ | /          \ | /
		//     \|/            \|/
		//     end

	Vertex* new_vtx = Vertex::allocate_from_pool(&mesh.vertex_pool, p);

	if (e->connect_face_array->size() == 2) {
		base_type::Face* f1 = (*(e->connect_face_array))[0];
		base_type::Face* f2 = (*(e->connect_face_array))[1];

		Vertex* v_orig = e->orig;
		Vertex* v_end = e->end;

		Vertex* v_t1 = base_type::Face::get_disjoin_face_no_share_vtx(f2, f1);
		Vertex* v_t2 = base_type::Face::get_disjoin_face_no_share_vtx(f1, f2);

		//        Edge *e_t11 = f1->disjoin_edge[base_type::Face::get_vtx_index(f1, v_end)];
		Edge* e_t12 = f1->disjoin_edge[base_type::Face::get_vtx_index(f1, v_orig)];

		//        Edge *e_t21 = f2->disjoin_edge[base_type::Face::get_vtx_index(f2, v_end)];
		Edge* e_t22 = f2->disjoin_edge[base_type::Face::get_vtx_index(f2, v_orig)];

		//new
		new_f1 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_end, new_vtx);
		new_f2 = Face::allocate_from_pool(&mesh.face_pool, v_t2, v_end, new_vtx);

		Edge* new_e0 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_end);
		Edge* new_e1 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t1);
		Edge* new_e2 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t2);

		//change e
		e->end = new_vtx;

		//change f1 and f2
		Face::set_vtx(f1, new_vtx, Face::get_vtx_index(f1, v_end));
		Face::set_vtx(f2, new_vtx, Face::get_vtx_index(f2, v_end));

		f1->disjoin_edge[Face::get_vtx_index(f1, v_orig)] = new_e1;
		f2->disjoin_edge[Face::get_vtx_index(f2, v_orig)] = new_e2;

		//change new face and edge
		(*(new_e0->connect_face_array)).push_back(new_f1);
		(*(new_e0->connect_face_array)).push_back(new_f2);
		(*(new_e1->connect_face_array)).push_back(f1);
		(*(new_e1->connect_face_array)).push_back(new_f1);
		(*(new_e2->connect_face_array)).push_back(f2);
		(*(new_e2->connect_face_array)).push_back(new_f2);

		new_f1->disjoin_edge[0] = new_e0;
		new_f2->disjoin_edge[0] = new_e0;
		new_f1->disjoin_edge[1] = new_e1;
		new_f2->disjoin_edge[1] = new_e2;
		new_f1->disjoin_edge[2] = e_t12;
		new_f2->disjoin_edge[2] = e_t22;

		Edge::del_connect_face(e_t12, f1);
		Edge::del_connect_face(e_t22, f2);
		Edge::add_connect_face(e_t12, new_f1);
		Edge::add_connect_face(e_t22, new_f2);

		interFaceAddArray.push_back(new_f1);
		interFaceAddArray.push_back(new_f2);
		interFaceArray.push_back(new_f1);
		interFaceArray.push_back(new_f2);

		std::array<Edge*, 2> new_edge({ new_e0, e });

		return { new_vtx, new_edge };
	}
	if (e->connect_face_array->size() == 1) {
		base_type::Face* f1 = (*(e->connect_face_array))[0];

		Vertex* v_orig = e->orig;
		Vertex* v_end = e->end;
		Vertex* v_t1 = nullptr;

		if (f1->p1 != v_orig && f1->p1 != v_end) {
			v_t1 = f1->p1;
		}
		if (f1->p2 != v_orig && f1->p2 != v_end) {
			v_t1 = f1->p2;
		}
		if (f1->p3 != v_orig && f1->p3 != v_end) {
			v_t1 = f1->p3;
		}

		//new
		new_f1 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_orig, new_vtx);
		new_f2 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_end, new_vtx);

		Edge* new_e0 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_orig);
		Edge* new_e1 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_end);
		Edge* new_e2 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t1);

		(*(new_e0->connect_face_array)).push_back(new_f1);
		(*(new_e1->connect_face_array)).push_back(new_f2);
		(*(new_e2->connect_face_array)).push_back(new_f1);
		(*(new_e2->connect_face_array)).push_back(new_f2);

		Edge* edge1 = Face::get_edge_from_two_vertex(f1, v_orig, v_t1);//f1->disjoin_edge[Face::get_vtx_index(f1, v_end)];
		Edge* edge2 = Face::get_edge_from_two_vertex(f1, v_end, v_t1);//f1->disjoin_edge[Face::get_vtx_index(f1, v_orig)];

		new_f1->disjoin_edge[0] = new_e0;
		new_f1->disjoin_edge[1] = new_e2;
		new_f1->disjoin_edge[2] = edge1;

		new_f2->disjoin_edge[0] = new_e1;
		new_f2->disjoin_edge[1] = new_e2;
		new_f2->disjoin_edge[2] = edge2;

		Edge::del_connect_face(edge1, f1);
		Edge::del_connect_face(edge2, f1);
		Edge::add_connect_face(edge1, new_f1);
		Edge::add_connect_face(edge2, new_f2);

		//            new_f1->mark = true;
		//            new_f2->mark = true;

		interFaceAddArray.push_back(new_f1);
		interFaceAddArray.push_back(new_f2);
		interFaceArray.push_back(new_f1);
		interFaceArray.push_back(new_f2);
		interFaceDeleteArray.push_back(f1);

		interFaceArray.erase(std::remove(interFaceArray.begin(), interFaceArray.end(), f1), interFaceArray.end());

		//delete
		mesh.face_pool.deallocate(f1);
		mesh.edge_pool.deallocate(e);



		std::array<Edge*, 2> new_edge({ new_e2, e });

		return { new_vtx, new_edge };
	}
	if (e->connect_face_array->size() == 0) {
		assert(false);
	}
	std::pair<Vertex*, std::array<Edge*, 2>> result_error(nullptr, {});
	return result_error;
	};

void tri_split(Triangle_Soup_Mesh& mesh, base_type::Face* f, base_type::Vector3& p, Face*& f1_new, Face*& f2_new, Face*& f3_new, Vertex*& new_vtx, vector<Face*>& interFaceArray, vector<Face*>& interFaceDeleteArray, vector<Face*>& interFaceAddArray) {
	double alpha, beta;
	Face* f11;
	Face* f22;

	std::tie(alpha, beta) = point_uv_calulate_triangle({ f->p1->position, f->p2->position, f->p3->position }, p);
	Vtx_state vtx_state = get_vtx_state(alpha, beta);

	if (vtx_state == OnEdge) {
		std::array<Edge*, 2> new_e1;
		Edge* e = abs(1 - alpha - beta) < 1e-6 ? f->disjoin_edge[0] : (abs(alpha) < 1e-6 ? f->disjoin_edge[1] : f->disjoin_edge[2]);
		std::tie(new_vtx, new_e1) = edge_split(mesh, e, p, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);
	}
	else if (vtx_state == Inside) {

		Vertex* p1 = f->p1;
		Vertex* p2 = f->p2;
		Vertex* p3 = f->p3;

		Edge* e1 = f->disjoin_edge[0];
		Edge* e2 = f->disjoin_edge[1];
		Edge* e3 = f->disjoin_edge[2];

		//creat
		new_vtx = Vertex::allocate_from_pool(&mesh.vertex_pool, p);

		f1_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p2, p3);
		f2_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p3, p1);
		f3_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p1, p2);

		Edge* e1_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p1);
		Edge* e2_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p2);
		Edge* e3_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p3);

		//change
		Edge::del_connect_face(e1, f);
		Edge::del_connect_face(e2, f);
		Edge::del_connect_face(e3, f);

		Edge::add_connect_face(e1, f1_new);
		Edge::add_connect_face(e2, f2_new);
		Edge::add_connect_face(e3, f3_new);

		Edge::add_connect_face(e1_new, f2_new);
		Edge::add_connect_face(e1_new, f3_new);

		Edge::add_connect_face(e2_new, f3_new);
		Edge::add_connect_face(e2_new, f1_new);

		Edge::add_connect_face(e3_new, f1_new);
		Edge::add_connect_face(e3_new, f2_new);

		f1_new->disjoin_edge[0] = e1;
		f1_new->disjoin_edge[1] = e3_new;
		f1_new->disjoin_edge[2] = e2_new;

		f2_new->disjoin_edge[0] = e2;
		f2_new->disjoin_edge[1] = e1_new;
		f2_new->disjoin_edge[2] = e3_new;

		f3_new->disjoin_edge[0] = e3;
		f3_new->disjoin_edge[1] = e2_new;
		f3_new->disjoin_edge[2] = e1_new;

		f1_new->mark = true;
		f2_new->mark = true;
		f3_new->mark = true;

		//delete

		interFaceAddArray.push_back(f1_new);
		interFaceAddArray.push_back(f2_new);
		interFaceAddArray.push_back(f3_new);
		interFaceArray.push_back(f1_new);
		interFaceArray.push_back(f2_new);
		interFaceArray.push_back(f3_new);
		interFaceDeleteArray.push_back(f);

		interFaceArray.erase(std::remove(interFaceArray.begin(), interFaceArray.end(), f), interFaceArray.end());

		mesh.face_pool.deallocate(f);
	}
	else {
		assert(false);
	}
	};

void clear_all_tri_mark(Triangle_Soup_Mesh& mesh) {
	for (int i = 0; i < mesh.face_pool.size(); i++) {
		base_type::Face* f_mesh = (base_type::Face*)mesh.face_pool[i];
		f_mesh->mark = false;
	}
	};

void tri_mark(std::vector<Face*>& f_array) {
	for (auto f : f_array) {
		f->mark = true;
	}
	};

Cut_result get_cut_result(base_type::Face* f_insert, const base_type::Vector3& p1, const base_type::Vector3& p2){

	double alpha_p1, beta_p1;
	double alpha_p2, beta_p2;
	std::tie(alpha_p1, beta_p1) = point_uv_calulate_triangle({ f_insert->p1->position, f_insert->p2->position, f_insert->p3->position }, p1);
	std::tie(alpha_p2, beta_p2) = point_uv_calulate_triangle({ f_insert->p1->position, f_insert->p2->position, f_insert->p3->position }, p2);

	Cut_result cut_result;

	Vtx_state p1_state = get_vtx_state(alpha_p1, beta_p1);
	Vtx_state p2_state = get_vtx_state(alpha_p2, beta_p2);

	// case 1 p1 and p2 inside f_mesh , split f_mesh
	if (p1_state == Inside && p2_state == Inside) {
		cut_result.cut_case = P1_P2_Inside;
	}
	else if (p1_state == Inside && p2_state == OnEdge) {
		cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.cut_case = P1_Inside_P2_OnEdge;
	}
	else if (p1_state == OnEdge && p2_state == Inside) {
		cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.cut_case = P1_OnEdge_P2_Inside;
	}
	else if (p1_state == OnEdge && p2_state == OnEdge) {
		cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.cut_case = P1_P2_OnEdge;
	}
	else if (p1_state == OnVtx && p2_state == Inside) {
		cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.cut_case = P1_OnVtx_P2_Inside;
	}
	else if (p1_state == Inside && p2_state == OnVtx) {
		cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.cut_case = P1_Inside_P2_OnVtx;
	}
	else if (p1_state == OnVtx && p2_state == OnEdge) {
		cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.cut_case = P1_OnVtx_P2_OnEdge;
	}
	else if (p1_state == OnEdge && p2_state == OnVtx) {
		cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
		cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.cut_case = P1_OnEdge_P2_OnVtx;
	}
	else if (p1_state == OnVtx && p2_state == OnVtx) {
		cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
		cut_result.cut_case = P1_P2_OnVtx;
	}
	else {
		assert(false);
	}
	return cut_result;
	};

void insert_one_tri(Triangle_Soup_Mesh& mesh, vector<Face*>& interFaceArray, base_type::Face* f_insert, vector<Face*>& interFaceDeleteArray, vector<Face*>& interFaceAddArray) {

	clear_all_tri_mark(mesh);

insert_start:
	for (int i = 0; i < interFaceArray.size(); i++) {

		base_type::Face* f_mesh = (base_type::Face*)interFaceArray[i];
		if (f_mesh->mark == true) {
			//interFaceArray.erase(std::remove(interFaceArray.begin(), interFaceArray.end(), f_mesh), interFaceArray.end());
			continue;
		}
		else {
			f_mesh->mark = true;
		}

		base_type::Vector3 p1;
		base_type::Vector3 p2;
		if (!tri_tri_cut(f_mesh, f_insert, p1, p2)) {
			continue;
		}

		auto c_r = get_cut_result(f_mesh, p1, p2);

		switch (c_r.cut_case) {
		case P1_P2_Inside: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			Face* f1, * f2, * f3;
			Face* f11, * f22, * f33;

			tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			Triangle tri = Triangle(f1->p1->position, f1->p2->position, f1->p3->position);
			bool flag = true;
			if (InTriangle(tri, p2) != -1 && flag) {
				tri_split(mesh, f1, p2, f11, f22, f33, new_v2, interFaceArray, interFaceDeleteArray, interFaceAddArray);

				flag = false;
			}
			tri = Triangle(f2->p1->position, f2->p2->position, f2->p3->position);
			if (InTriangle(tri, p2) != -1 && flag) {
				tri_split(mesh, f2, p2, f11, f22, f33, new_v2, interFaceArray, interFaceDeleteArray, interFaceAddArray);

				flag = false;
			}
			tri = Triangle(f3->p1->position, f3->p2->position, f3->p3->position);
			if (InTriangle(tri, p2) != -1 && flag) {
				tri_split(mesh, f3, p2, f11, f22, f33, new_v2, interFaceArray, interFaceDeleteArray, interFaceAddArray);

				flag = false;
			}

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			//tri_mark(*(edge_find->connect_face_array));

			new_v1->special = true;
			new_v2->special = true;
			break;
		}
		case P1_Inside_P2_OnEdge: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			Face* f1, * f2, * f3;
			Face* f11, * f22;
			std::array<Edge*, 2> new_e1;

			tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			std::tie(new_v2, new_e1) = edge_split(mesh, c_r.c_e[0], p2, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);


			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			tri_mark(*(edge_find->connect_face_array));
			new_v1->special = true;
			new_v2->special = true;
			break;
		}
		case P1_OnEdge_P2_Inside: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			Face* f1, * f2, * f3;
			Face* f11, * f22;
			std::array<Edge*, 2> new_e1;

			tri_split(mesh, f_mesh, p2, f1, f2, f3, new_v1, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			std::tie(new_v2, new_e1) = edge_split(mesh, c_r.c_e[0], p1, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			tri_mark(*(edge_find->connect_face_array));
			new_v1->special = true;
			new_v2->special = true;
			break;
		}

		case P1_P2_OnEdge: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			std::array<Edge*, 2> new_e1, new_e2;
			Face* f11, * f22, * f33, * f44;

			std::tie(new_v1, new_e1) = edge_split(mesh, c_r.c_e[0], p1, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);
			std::tie(new_v2, new_e2) = edge_split(mesh, c_r.c_e[1], p2, f33, f44, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			tri_mark(*(edge_find->connect_face_array));
			new_v1->special = true;
			new_v2->special = true;
			break;
		}

		case P1_OnVtx_P2_Inside: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			Face* f1, * f2, * f3;

			if (p1.distance(f_mesh->p1->position) < 1e-6) {
				new_v1 = f_mesh->p1;
			}
			if (p1.distance(f_mesh->p2->position) < 1e-6) {
				new_v1 = f_mesh->p2;
			}
			if (p1.distance(f_mesh->p3->position) < 1e-6) {
				new_v1 = f_mesh->p3;
			}

			tri_split(mesh, f_mesh, p2, f1, f2, f3, new_v2, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			//                    tri_mark(*(edge_find->connect_face_array));
			new_v1->special = true;
			new_v2->special = true;
			break;
		}
		case P1_Inside_P2_OnVtx: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			Face* f1, * f2, * f3;

			if (p2.distance(f_mesh->p1->position) < 1e-6) {
				new_v2 = f_mesh->p1;
			}
			if (p2.distance(f_mesh->p2->position) < 1e-6) {
				new_v2 = f_mesh->p2;
			}
			if (p2.distance(f_mesh->p3->position) < 1e-6) {
				new_v2 = f_mesh->p3;
			}

			tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			edge_find->special = true;
			//                    tri_mark(*(edge_find->connect_face_array));
			new_v1->special = true;
			new_v2->special = true;
			break;
		}

		case P1_OnVtx_P2_OnEdge: {
			Vertex* new_v = nullptr;
			std::array<Edge*, 2> new_e;
			Face* f11, * f22;
			std::tie(new_v, new_e) = edge_split(mesh, c_r.c_e[1], p2, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v, c_r.c_v[0]);
			edge_find->special = true;
			tri_mark(*(edge_find->connect_face_array));
			new_v->special = true;
			break;
		}
		case P1_OnEdge_P2_OnVtx: {
			Vertex* new_v;
			std::array<Edge*, 2> new_e;
			Face* f11, * f22;
			std::tie(new_v, new_e) = edge_split(mesh, c_r.c_e[0], p1, f11, f22, interFaceArray, interFaceDeleteArray, interFaceAddArray);

			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v, c_r.c_v[1]);
			edge_find->special = true;
			tri_mark(*(edge_find->connect_face_array));
			edge_find->special = true;
			new_v->special = true;
			break;
		}

		case P1_P2_OnVtx: {
			Vertex* new_v1 = nullptr;
			Vertex* new_v2 = nullptr;
			new_v1 = c_r.c_v[0];
			new_v2 = c_r.c_v[1];
			auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
			tri_mark(*(edge_find->connect_face_array));
			edge_find->special = true;

			new_v1->special = true;
			new_v2->special = true;
			break;
		}

		case No_Cut: {
			break;
		}
		default:
			assert(false);
		}

		goto insert_start;
	}
	};

void surface_cut(std::string Path, base_type::Triangle_Soup_Mesh& meshCube, base_type::Triangle_Soup_Mesh& meshCurve, base_type::Triangle_Soup_Mesh& meshResult, int index) {

	logger().info("Compute Start");

	Triangle_Soup_Mesh meshCube2;
	Triangle_Soup_Mesh meshCurve2;

	meshCube2.copy(meshCube);

	//step 1: use meshCurve to subdivide meshCube

	std::vector<Object*> objects_meshCube;
	for (int i = 0; i < meshCube.face_pool.size(); i++) {
		auto f = (base_type::Face*)meshCube.face_pool[i];
		Object* obj = new Object{ computeAABB(*f), f };
		objects_meshCube.push_back(obj);
	}

	BVHNode* root_meshCube = buildBVH(objects_meshCube);
	
	vector<Face*> interFaceDeleteArray;
	vector<Face*> interFaceAddArray;

	for (int i = 0; i < meshCurve.face_pool.size(); i++) {
		auto f = (base_type::Face*)meshCurve.face_pool[i];
		vector<Face*> interFaceArray;
		bool DeleteFlag = false;
		AABB_intersection(root_meshCube, f, interFaceArray, interFaceDeleteArray, DeleteFlag);
		//if (interFaceArray.size() == 0 && !DeleteFlag) {
		//	continue;
		//}
		for (auto interFaceAdd : interFaceAddArray) {
			Vector3 p1, p2;
			if (tri_tri_cut(interFaceAdd, f, p1, p2)) {
				interFaceArray.push_back(interFaceAdd);
			}
		}
		if (interFaceArray.size() != 0) {
			insert_one_tri(meshCube, interFaceArray, f, interFaceDeleteArray, interFaceAddArray);
		}
	}

	meshCube.save(Path, "outputCube");

	std::vector<Object*> objects_meshCurve;
	for (int i = 0; i < meshCurve.face_pool.size(); i++) {
		auto f = (base_type::Face*)meshCurve.face_pool[i];
		Object* obj = new Object{ computeAABB(*f), f };
		objects_meshCurve.push_back(obj);
	}

	BVHNode* root_meshCurve = buildBVH(objects_meshCurve);

	interFaceDeleteArray.clear();
	interFaceAddArray.clear();

	for (int i = 0; i < meshCube2.face_pool.size(); i++) {
		auto f = (base_type::Face*)meshCube2.face_pool[i];
		vector<Face*> interFaceArray;
		bool DeleteFlag = false;
		AABB_intersection(root_meshCurve, f, interFaceArray, interFaceDeleteArray, DeleteFlag);
		//if (interFaceArray.size() == 0 && !DeleteFlag) {
		//	continue;
		//}
		for (auto interFaceAdd : interFaceAddArray){
			Vector3 p1, p2;
			if (tri_tri_cut(interFaceAdd, f, p1, p2)) {
				interFaceArray.push_back(interFaceAdd);
			}
		}

		if (interFaceArray.size() != 0) {
			insert_one_tri(meshCurve, interFaceArray, f, interFaceDeleteArray, interFaceAddArray);
		}
	}

	meshCurve.save(Path, "outputCurve");

	//step 2: depart mesh by special edge
	auto get_unmarked_face = [](Triangle_Soup_Mesh& mesh) -> Face* {
		for (int i = 0; i < mesh.face_pool.size(); i++) {
			base_type::Face* f = (base_type::Face*)mesh.face_pool[i];
			if (f->mark == false)
				return f;
		}
		return nullptr;
		};

	clear_all_tri_mark(meshCube);
	clear_all_tri_mark(meshCurve);

	auto unmarked_face = get_unmarked_face(meshCube);
	auto unmarked_face_Curve = get_unmarked_face(meshCurve);

	int part_index = 0;
	do {
		std::vector<Face*> face_stack = { unmarked_face_Curve };
		std::vector<Face*> face_array;

		while (!face_stack.empty()) {
			auto back = face_stack.back();
			face_stack.pop_back();
			back->mark = true;
			face_array.push_back(back);
			for (int i = 0; i < 3; i++) {
				if (back->disjoin_edge[i]->special == false && back->disjoin_edge[i]->connect_face_array->size() == 2) {
					auto f = Face::get_disjoin_face(back, back->disjoin_edge[i]);
					if (!f->mark)
						face_stack.push_back(f);
				}
			}
		}

		unmarked_face_Curve = get_unmarked_face(meshCurve);

		Triangle_Soup_Mesh part;

		for (auto f : face_array) {
			//auto v1 = part.add_vtx(f->p1->position);
			//auto v2 = part.add_vtx(f->p2->position);
			//auto v3 = part.add_vtx(f->p3->position);
			//part.add_face(v1, v2, v3);
			auto v1 = Vertex::allocate_from_pool(&part.vertex_pool, f->p1->position);
			auto v2 = Vertex::allocate_from_pool(&part.vertex_pool, f->p2->position);
			auto v3 = Vertex::allocate_from_pool(&part.vertex_pool, f->p3->position);
			Face::allocate_from_pool(&part.face_pool, v1, v2, v3);
		}
		if (part_index == index) {
			meshCurve2.copy_no_edge(part);
		}
		part.save(Path, "output_Curve" + std::to_string(part_index++));
	} while (unmarked_face_Curve);

	//meshCurve2.load_from_file("D:/xmy/model/output_Curve1.obj");

	part_index = 0;
	do {
		std::vector<Face*> face_stack = { unmarked_face };
		std::vector<Face*> face_array;

		while (!face_stack.empty()) {
			auto back = face_stack.back();
			face_stack.pop_back();
			back->mark = true;
			face_array.push_back(back);
			for (int i = 0; i < 3; i++) {
				if (back->disjoin_edge[i]->special == false) {
					auto f = Face::get_disjoin_face(back, back->disjoin_edge[i]);
					if (!f->mark)
						face_stack.push_back(f);
				}
			}
		}

		unmarked_face = get_unmarked_face(meshCube);

		Triangle_Soup_Mesh part;

		for (auto f : face_array) {
			//auto v1 = part.add_vtx(f->p1->position);
			//auto v2 = part.add_vtx(f->p2->position);
			//auto v3 = part.add_vtx(f->p3->position);
			//part.add_face(v1, v2, v3);
			auto v1 = Vertex::allocate_from_pool(&part.vertex_pool, f->p1->position);
			auto v2 = Vertex::allocate_from_pool(&part.vertex_pool, f->p2->position);
			auto v3 = Vertex::allocate_from_pool(&part.vertex_pool, f->p3->position);
			Face::allocate_from_pool(&part.face_pool, v1, v2, v3);
		}

		for (int i = 0; i < meshCurve2.face_pool.size(); i++) {
			auto f = (base_type::Face*)meshCurve2.face_pool[i];
			//auto v1 = part.add_vtx(f->p1->position);
			//auto v2 = part.add_vtx(f->p2->position);
			//auto v3 = part.add_vtx(f->p3->position);
			//part.add_face(v1, v2, v3);
			auto v1 = Vertex::allocate_from_pool(&part.vertex_pool, f->p1->position);
			auto v2 = Vertex::allocate_from_pool(&part.vertex_pool, f->p2->position);
			auto v3 = Vertex::allocate_from_pool(&part.vertex_pool, f->p3->position);
			Face::allocate_from_pool(&part.face_pool, v1, v2, v3);
		}
		meshResult.copy(part);
		part.save(Path, "output_Cube" + std::to_string(part_index++));
	} while (unmarked_face);

	logger().info("end");

}



