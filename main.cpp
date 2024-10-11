#include <vector>
#include <algorithm>
#include <maya/MPxCommand.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MDagPath.h>
#include <maya/MColorArray.h>
#include <maya/MPointArray.h>
#include <maya/MObject.h>
#include <maya/MEventMessage.h>
#include <maya/MMessage.h>
#include <maya/MBoundingBox.h>
#include <maya/MFnMesh.h>
#include <maya/MColor.h>
#include <maya/MObjectHandle.h>
#include <maya/MDGMessage.h>
#include <memory>

// �O�όv�Z.
MVector cross(const MVector& a, const MVector& b) {
	MVector ans;
	ans.x = a.y * b.z - a.z * b.y;
	ans.y = a.z * b.x - a.x * b.z;
	ans.z = a.x * b.y - a.y * b.x;
	return ans;
}

// ���όv�Z.
double dot(const MVector& a, const MVector& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// ��ԕ����@�Ńo�C�i���\�����쐬����ۂɗp����N���X.
class BVHnode {
public:
	MBoundingBox aabb; // AABB.
	//�񕪖�.
	std::unique_ptr<BVHnode> left;
	std::unique_ptr<BVHnode> right;

	// ���̃m�[�h�ȉ��ɂ���|���S���̃��X�g.
	std::vector<int> polyList;
};


// �w�肵���|���S����AABB���v�Z����֐�.
MBoundingBox calcPolyAABB(const MFnMesh& mesh, int polyIndex) {
	MItMeshPolygon polyIter(mesh.object());
	int prevIndex = 0;
	polyIter.setIndex(polyIndex, prevIndex);

	MPointArray vertices;
	polyIter.getPoints(vertices);

	MBoundingBox aabb;
	for (int i = 0; i < (int)(vertices.length()); ++i) {
		aabb.expand(vertices[i]);
	}

	return aabb;

}

// �������ɋϓ��ɐ؂���, AABB�̃o�C�i������������֐�.
std::unique_ptr<BVHnode> makeBVHtree(std::vector<int>& polyList, const MFnMesh& mesh, int depth = 0) {
	// polyList����̂Ƃ���, nullptr��Ԃ�.
	if ((int)(polyList.size()) == 0) {
		return nullptr;
	}

	std::unique_ptr<BVHnode> node = std::make_unique<BVHnode>();

	// �|���S�����X�g�̃|���S�������ׂĊ܂�AABB���쐬.
	for (int poly : polyList) {
		MItMeshPolygon polyIter(mesh.object());
		int polyIndex = -1;

		for (; !polyIter.isDone(); polyIter.next()) {
			if (polyIter.index() == poly) {
				polyIndex = polyIter.index();
				break;
			}
		}

		if (polyIndex == -1) {
			MGlobal::displayError("�|���S�����������܂���.");
			return nullptr;
		}

		MBoundingBox aabb = calcPolyAABB(mesh, polyIndex);
		// ����node��AABB����̏ꍇ, �����AABB�ɐݒ�.
		if (node->aabb.width() == 0 &&
			node->aabb.height() == 0 &&
			node->aabb.depth() == 0) {
			node->aabb = aabb;
		}
		else {
			node->aabb.expand(aabb);
		}
	}

	if ((int)(polyList.size()) > 1) {
		int axis = depth % 3; // 0: x, 1: y, 2: z.

		std::sort(polyList.begin(), polyList.end(), [&mesh, axis](int& polyA, int& polyB) {

			MItMeshPolygon polyIter(mesh.object());
			int polyAIndex = -1;
			int polyBIndex = -1;

			for (; !polyIter.isDone(); polyIter.next()) {
				if (polyIter.index() == polyA) {
					polyAIndex = polyIter.index();
				}
				else if (polyIter.index() == polyB) {
					polyBIndex = polyIter.index();
				}

				if (polyAIndex != -1 && polyBIndex != -1) {
					break;
				}
			}

			MBoundingBox boxA = calcPolyAABB(mesh, polyAIndex);
			MBoundingBox boxB = calcPolyAABB(mesh, polyBIndex);

			if (axis == 0) {
				return boxA.center().x < boxB.center().x;
			}
			else if (axis == 1) {
				return boxA.center().y < boxB.center().y;
			}
			else {
				return boxA.center().z < boxB.center().z;
			}
			}
		);

		size_t mid = polyList.size() / 2;

		std::vector<int> leftMeshList(polyList.begin(), polyList.begin() + mid);
		std::vector<int> rightMeshList(polyList.begin() + mid, polyList.end());

		node->left = makeBVHtree(leftMeshList, mesh, depth + 1);
		node->right = makeBVHtree(rightMeshList, mesh, depth + 1);
	}

	// node���t�̏ꍇ��, ���̃m�[�h��mesh�����i�[.
	else {
		node->polyList.emplace_back(polyList[0]);
	}

	return node;
}

// �ʂƖʂ���������Ă���Ȃ�true��Ԃ��֐�.
bool intersectJudge(const MPointArray& vertices1, const MPointArray& vertices2) {
	MVector n1, n2;
	MVector vec11(vertices1[1] - vertices1[0]);
	MVector vec12(vertices1[2] - vertices1[0]);
	n1 = cross(vec11, vec12);
	MVector vec21(vertices2[1] - vertices2[0]);
	MVector vec22(vertices2[2] - vertices2[0]);
	n2 = cross(vec21, vec22);

	MVector dirVec = cross(n1, n2); // ����̕����x�N�g��.
	if (dirVec.x == 0 && dirVec.y == 0 && dirVec.z == 0) return false;
	MVector p; // �����̂����_(z = 0).
	double num1 = n1.x * vertices1[0].x + n1.y * vertices1[0].y + n1.z * vertices1[0].z;
	double num2 = n2.x * vertices2[0].x + n2.y * vertices2[0].y + n2.z * vertices2[0].z;
	double num3 = n1.x * n2.z - n1.z * n2.x;
	double num4 = n1.y * n2.z - n1.z * n2.y;

	if (num3 == 0) {
		if (num4 == 0) {
			return false;
		}
		else {
			p.x = 0;
			p.y = (num1 * n2.z - num2 * n1.z) / num4;
		}
	}
	else {
		p.x = (num1 * n2.z - num2 * n1.z) / num3;
		p.y = 0;
	}

	if (n1.z != 0) {
		p.z = (num1 - n1.x * p.x - n1.y * p.y) / n1.z;
	}
	else if (n2.z != 0) {
		p.z = (num2 - n2.x * p.x - n2.y * p.y) / n2.z;
	}
	else {
		p.z = 0;
	}

	bool flag1 = false; // ����̖ʂɑ΂���, ������ʏ�(�ӏ㏜��)�ɂ���Ȃ�false.
	bool flag2 = false;
	if ((int)(vertices1.length()) == 3) {
		MVector v1 = cross((p - vertices1[0]), dirVec);
		MVector v2 = cross((p - vertices1[1]), dirVec);
		MVector v3 = cross((p - vertices1[2]), dirVec);

		flag1 = ((dot(v1, v2) > 0 && dot(v1, v3) > 0 && dot(v2, v3) > 0)
			|| (dot(v1, v2) < 0 && dot(v1, v3) < 0 && dot(v2, v3) < 0)) ||
			dot(v1, v2) * dot(v1, v3) * dot(v2, v3) == 0;

		v1 = cross((p - vertices2[0]), dirVec);
		v2 = cross((p - vertices2[1]), dirVec);
		v3 = cross((p - vertices2[2]), dirVec);

		flag2 = ((dot(v1, v2) > 0 && dot(v1, v3) > 0 && dot(v2, v3) > 0)
			|| (dot(v1, v2) < 0 && dot(v1, v3) < 0 && dot(v2, v3) < 0));

		if (!flag1 && !flag2) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		MVector v1 = cross((p - vertices1[0]), dirVec);
		MVector v2 = cross((p - vertices1[1]), dirVec);
		MVector v3 = cross((p - vertices1[2]), dirVec);
		MVector v4 = cross((p - vertices1[3]), dirVec);

		flag1 = ((dot(v1, v2) > 0 && dot(v1, v3) > 0 && dot(v1, v4) > 0 && dot(v2, v3) > 0)
			|| (dot(v1, v2) < 0 && dot(v1, v3) < 0 && dot(v1, v4) < 0 && dot(v2, v3) < 0) ||
			dot(v1, v2) * dot(v1, v3) * dot(v1, v4) * dot(v2, v3) == 0);

		v1 = cross((p - vertices2[0]), dirVec);
		v2 = cross((p - vertices2[1]), dirVec);
		v3 = cross((p - vertices2[2]), dirVec);
		v4 = cross((p - vertices2[3]), dirVec);

		flag2 = ((dot(v1, v2) > 0 && dot(v1, v3) > 0 && dot(v1, v4) > 0 && dot(v2, v3) > 0)
			|| (dot(v1, v2) < 0 && dot(v1, v3) < 0 && dot(v1, v4) < 0 && dot(v2, v3) < 0) ||
			dot(v1, v2) * dot(v1, v3) * dot(v1, v4) * dot(v2, v3) == 0);

		// 2�̖ʂɑ΂���, �ǂ����������ʏ�(�ӏ㏜��)�ɂȂ��Ƃ�true.
		if (!flag1 && !flag2) {
			return true;
		}
		else {
			return false;
		}
	}
}

// ����AABB�ɑ΂���, BVH�̃c���[����������Ă���MObject��intersectPoly�Ɋi�[����֐�.
void calcIntersectPoly(const MFnMesh& mesh, const BVHnode& node, const MPointArray& vertices, std::vector<int>& intersectPoly) {
	MBoundingBox baseAABB;
	for (int i = 0; i < (int)(vertices.length()); ++i) {
		baseAABB.expand(vertices[i]);
	}

	bool flag = node.aabb.intersects(baseAABB);
	// AABB�̌�������.
	if (!flag) {
		return;
	}

	// AABB���������Ă��� + node���t�̂Ƃ�.
	else if (flag && (int)(node.polyList.size()) == 1) {
		// �ʓ��m���������Ă�����intersectPoly�z��ɖʏ����i�[.
		MItMeshPolygon polyIter(mesh.object());
		int prevIndex;
		polyIter.setIndex(node.polyList[0], prevIndex);

		MPointArray polyVertices;
		polyIter.getPoints(polyVertices);

		if (intersectJudge(polyVertices, vertices)) {
			intersectPoly.push_back(node.polyList[0]);
		}

		return;
	}

	else {
		calcIntersectPoly(mesh, *node.left, vertices, intersectPoly);
		calcIntersectPoly(mesh, *node.right, vertices, intersectPoly);
	}
}

// intersectPoly�Ŏw�肵���|���S���ɐF��h��֐�.
void setPolyColor(MFnMesh& mesh, MDagPath& dagPath, MObject& component, std::vector<int>& intersectPoly) {
	MString colorSetName = "IntersectSetColor";
	MStringArray colorNames;
	mesh.getColorSetNames(colorNames);

	if ((int)(colorNames.length()) == 0) {
		mesh.createColorSet(colorSetName);
		mesh.setCurrentColorSetName(colorSetName);
		MGlobal::displayInfo("�J���[�Z�b�g���쐬���܂���.");
	}

	MColor color(1.0f, 0.0f, 0.0f); // �ԐF.

	// ���b�V���ɂ�����e�|���S���ɑ΂��ĐF��h��.
	MItMeshPolygon polyIter(dagPath, component);

	for (int poly : intersectPoly) {
		mesh.setFaceColor(color, poly);
	}

	mesh.updateSurface();
}

// �J���[�Z�b�g�̉���.
void resetPolyColor() {
	MSelectionList selection;
	MGlobal::getActiveSelectionList(selection);
	MItSelectionList selectIter(selection);

	MDagPath dagPath;

	std::vector<int> polyList;

	//���݂͈ꕨ�̂ɑ΂���R�[�h.
	for (; !selectIter.isDone(); selectIter.next()) {
		MDagPath dagPath;
		MObject component;
		selectIter.getDagPath(dagPath, component);
		MFnMesh mesh(dagPath);
		MStatus status = mesh.deleteColorSet("IntersectSetColor");
	}
}


class IntersectJudge : public MPxCommand {
public:
	IntersectJudge() {}
	~IntersectJudge() {}

	static void* creator();

	static void registerCallback();
	static void deregisterCallback();

	static void callbackFunc(void* hoge);

private:
	static MCallbackId callbackId;
	static bool activeFlag;
};

MCallbackId IntersectJudge::callbackId = 0;
bool IntersectJudge::activeFlag = false;

void* IntersectJudge::creator() {
	return new IntersectJudge();
}

void IntersectJudge::registerCallback() {
	if (callbackId == 0) {
		callbackId = MEventMessage::addEventCallback("SelectionChanged", callbackFunc, nullptr);
		MGlobal::displayInfo("�R�[���o�b�N�֐����o�^����܂���.");
		activeFlag = true;
	}
	else {
		MGlobal::displayInfo("�G���[: �R�[���o�b�N�֐��̓o�^�Ɏ��s���܂���.");
	}

	return;
}

void IntersectJudge::deregisterCallback() {
	if (callbackId != 0) {
		MMessage::removeCallback(callbackId);
		callbackId = 0;
		MGlobal::displayInfo("�R�[���o�b�N�֐�����������܂���.");
		activeFlag = false;
	}
	else {
		MGlobal::displayError("�G���[: �R�[���o�b�N�֐��̉����Ɏ��s���܂���");
	}

	return;
}

void IntersectJudge::callbackFunc(void* hoge) {
	resetPolyColor();

	if (!activeFlag) {
		return;
	}

	MSelectionList selection;
	MGlobal::getActiveSelectionList(selection);
	MItSelectionList selectIter(selection);

	bool colorSetFlag = false;

	std::vector<int> polyList;
	std::unique_ptr<BVHnode> BVHtree;
	std::vector<int> intersectPoly;


	//���݂͈ꕨ�̂ɑ΂���R�[�h.
	for (; !selectIter.isDone(); selectIter.next()) {
		MDagPath dagPath;
		MObject component;
		selectIter.getDagPath(dagPath, component);
		MFnMesh mesh(dagPath);

		if (!dagPath.hasFn(MFn::kMesh) || !component.hasFn(MFn::kMeshPolygonComponent)) {
			continue;
		}

		else {
			polyList.clear();

			MItMeshPolygon polyIter(dagPath, component);

			for (; !polyIter.isDone(); polyIter.next()) {
				int polyIndex = polyIter.index();
				polyList.push_back(polyIndex);
			}
			if ((int)(polyList.size()) == 1) {
				continue;
			}

			BVHtree = std::move(makeBVHtree(polyList, mesh)); // BVH�c���[�쐬.

			polyIter.reset();

			for (; !polyIter.isDone(); polyIter.next()) {
				intersectPoly.clear();

				MPointArray vertices;
				polyIter.getPoints(vertices);

				calcIntersectPoly(mesh, *BVHtree, vertices, intersectPoly); // ���������|���S���̌���.

				setPolyColor(mesh, dagPath, component, intersectPoly); // ���̃|���S���̐F��ς���.
			}
		}
	}
}

MStatus initializePlugin(MObject obj) {
	MFnPlugin plugin(obj, "Kai", "v.2024.10.11", "2024");

	MStatus status = plugin.registerCommand("Intersect", IntersectJudge::creator);
	if (status) {
		MGlobal::displayInfo("Intersect�R�}���h��ǉ����܂���.");
	}

	IntersectJudge::registerCallback();

	return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);

	// �v���O�C���̑I��������.

	IntersectJudge::deregisterCallback();

	resetPolyColor();

	MStatus status = plugin.deregisterCommand("Intersect");

	if (status) {
		MGlobal::displayInfo("Intersect�R�}���h���������܂���.");
	}

	return MS::kSuccess;
}