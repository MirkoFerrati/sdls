void BuildTreeYShape(Node *node[], Tree &tree)
{
    const KDL::Vector unitx(1,0,0);
    const KDL::Vector unity(0,1,0);
    const KDL::Vector unitz(0,0,1);
    const KDL::Vector unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
    const KDL::Vector zero = KDL::Vector::Zero();
    
    //node[0] = new Node(KDL::Vector(0.0f, -0.5f, 0.0f), unit1, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    node[0] = new Node(KDL::Vector(0.0f, -0.5f, 0.0f), unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRoot(node[0]);
    
    node[1] = new Node(KDL::Vector(0.0f, 0.4f, 0.0f), unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[0], node[1]);
    
    node[2] = new Node(KDL::Vector(0.0f, 0.4f, 0.0f), unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRightSibling(node[1], node[2]);
    
    node[3] = new Node(KDL::Vector(0.5f, 1.0f, 0.0f), unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[1], node[3]);
    
    node[4] = new Node(KDL::Vector(-0.5f, 1.0f, 0.0f), unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[2], node[4]);
    
    node[5] = new Node(KDL::Vector(0.7f, 1.3f, 0.0f), unit1, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[3], node[5]);
    
    node[6] = new Node(KDL::Vector(-0.8f, 1.5f, 0.0f), unit1, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[4], node[6]);
    
    node[7] = new Node(KDL::Vector(0.7f, 2.0f, 0.0f), zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[5], node[7]);
    
    node[8] = new Node(KDL::Vector(-0.8f, 1.9f, 0.0f), zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[6], node[8]);
}

void BuildTreeDoubleYShape(Node *node[], Tree &tree)
{
    const KDL::Vector unitx(1,0,0);
    const KDL::Vector unity(0,1,0);
    const KDL::Vector unitz(0,0,1);
    const KDL::Vector unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
    const KDL::Vector zero = KDL::Vector::Zero();
    KDL::Vector p0(0.0f, -1.5f, 0.0f);
    KDL::Vector p1(0.0f, -1.0f, 0.0f);
    KDL::Vector p2(0.0f, -0.5f, 0.0f);
    KDL::Vector p3(0.5f*Root2Inv, -0.5+0.5*Root2Inv, 0.0f);
    KDL::Vector p4(0.5f*Root2Inv+0.5f*HalfRoot3, -0.5+0.5*Root2Inv+0.5f*0.5, 0.0f);
    KDL::Vector p5(0.5f*Root2Inv+1.0f*HalfRoot3, -0.5+0.5*Root2Inv+1.0f*0.5, 0.0f);
    KDL::Vector p6(0.5f*Root2Inv+1.5f*HalfRoot3, -0.5+0.5*Root2Inv+1.5f*0.5, 0.0f);
    KDL::Vector p7(0.5f*Root2Inv+0.5f*HalfRoot3, -0.5+0.5*Root2Inv+0.5f*HalfRoot3, 0.0f);
    KDL::Vector p8(0.5f*Root2Inv+1.0f*HalfRoot3, -0.5+0.5*Root2Inv+1.0f*HalfRoot3, 0.0f);
    KDL::Vector p9(0.5f*Root2Inv+1.5f*HalfRoot3, -0.5+0.5*Root2Inv+1.5f*HalfRoot3, 0.0f);
    KDL::Vector p10(-0.5f*Root2Inv, -0.5+0.5*Root2Inv, 0.0f);
    KDL::Vector p11(-0.5f*Root2Inv-0.5f*HalfRoot3, -0.5+0.5*Root2Inv+0.5f*HalfRoot3, 0.0f);
    KDL::Vector p12(-0.5f*Root2Inv-1.0f*HalfRoot3, -0.5+0.5*Root2Inv+1.0f*HalfRoot3, 0.0f);
    KDL::Vector p13(-0.5f*Root2Inv-1.5f*HalfRoot3, -0.5+0.5*Root2Inv+1.5f*HalfRoot3, 0.0f);
    KDL::Vector p14(-0.5f*Root2Inv-0.5f*HalfRoot3, -0.5+0.5*Root2Inv+0.5f*0.5, 0.0f);
    KDL::Vector p15(-0.5f*Root2Inv-1.0f*HalfRoot3, -0.5+0.5*Root2Inv+1.0f*0.5, 0.0f);
    KDL::Vector p16(-0.5f*Root2Inv-1.5f*HalfRoot3, -0.5+0.5*Root2Inv+1.5f*0.5, 0.0f);
    
    node[0] = new Node(p0, unit1, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRoot(node[0]);
    
    node[1] = new Node(p1, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[0], node[1]);
    
    node[2] = new Node(p1, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[1], node[2]);
    
    node[3] = new Node(p2, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[2], node[3]);
    
    node[4] = new Node(p2, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRightSibling(node[3], node[4]);
    
    node[5] = new Node(p3, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[3], node[5]);
    
    node[6] = new Node(p3, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRightSibling(node[5], node[6]);
    
    node[7] = new Node(p3, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[5], node[7]);
    
    node[8] = new Node(p4, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[7], node[8]);
    
    node[9] = new Node(p5, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[8], node[9]);
    
    node[10] = new Node(p5, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[9], node[10]);
    
    node[11] = new Node(p6, zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[10], node[11]);
    
    node[12] = new Node(p3, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[6], node[12]);
    
    node[13] = new Node(p7, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[12], node[13]);
    
    node[14] = new Node(p8, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[13], node[14]);
    
    node[15] = new Node(p8, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[14], node[15]);
    
    node[16] = new Node(p9, zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[15], node[16]);
    
    node[17] = new Node(p10, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[4], node[17]);
    
    node[18] = new Node(p10, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[17], node[18]);
    
    node[19] = new Node(p10, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertRightSibling(node[17], node[19]);
    
    node[20] = new Node(p11, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[18], node[20]);
    
    node[21] = new Node(p12, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[20], node[21]);
    
    node[22] = new Node(p12, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[21], node[22]);
    
    node[23] = new Node(p13, zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[22], node[23]);
    
    node[24] = new Node(p10, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[19], node[24]);
    
    node[25] = new Node(p14, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[24], node[25]);
    
    node[26] = new Node(p15, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[25], node[26]);
    
    node[27] = new Node(p15, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
    tree.InsertLeftChild(node[26], node[27]);
    
    node[28] = new Node(p16, zero, 0.08, EFFECTOR);
    tree.InsertLeftChild(node[27], node[28]);
}
