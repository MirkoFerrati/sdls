
#ifndef _CLASS_NODE
#define _CLASS_NODE
#include <kdl/frames.hpp>
#include "LinearR3.h"

enum Purpose {JOINT, EFFECTOR};


class Node {

	friend class Tree;

public:
	Node(const KDL::Vector&, const KDL::Vector&, double, Purpose, double minTheta=-PI, double maxTheta=PI, double restAngle=0.);

	void DrawNode(bool);
	void PrintNode();
	void InitNode();

	const KDL::Vector& GetAttach() const { return attach; }

	double GetTheta() const { return theta; }
	double AddToTheta( double delta ) { theta += delta; return theta; }

	const KDL::Vector& GetS() const { return s; }
	const KDL::Vector& GetW() const { return w; }

	double GetMinTheta() const { return minTheta; }
	double GetMaxTheta() const { return maxTheta; } 
	double GetRestAngle() const { return restAngle; } ;
	void SetTheta(double newTheta) { theta = newTheta; }
	void ComputeS(void);
	void ComputeW(void);

	bool IsEffector() const { return purpose==EFFECTOR; } 
	bool IsJoint() const { return purpose==JOINT; }
	int GetEffectorNum() const { return seqNumEffector; }
	int GetJointNum() const { return seqNumJoint; }

	bool IsFrozen() const { return freezed; }
	void Freeze() { freezed = true; }
	void UnFreeze() { freezed = false; }

private:
	bool freezed;			// Is this node frozen?
	int seqNumJoint;		// sequence number if this node is a joint
	int seqNumEffector;		// sequence number if this node is an effector
	double size;			// size
	Purpose purpose;		// joint / effector / both
	KDL::Vector attach;		// attachment point
	KDL::Vector r;				// relative position vector
	KDL::Vector v;				// rotation axis
	double theta;			// joint angle (radian)
	double minTheta;		// lower limit of joint angle
	double maxTheta;		// upper limit of joint angle
	double restAngle;		// rest position angle
	KDL::Vector s;				// GLobal Position
	KDL::Vector w;				// Global rotation axis
	Node* left;				// left child
	Node* right;			// right sibling
	Node* realparent;		// pointer to real parent

	void DrawBox() const;
};

#endif