
void RunTest () 
{
    RunTestB();
}

void RunTestC() 
{
    
    Node* nodesA[MAX_NUM_NODE];
    Tree treeA;
    BuildTreeDoubleYShape( nodesA, treeA );
    Jacobian jacobA( &treeA );
    
    double time = 0.0;
    while ( true ) {
        time += 3.0*(1.0+sqrt(5.0));
        treeA.Init();
        treeA.Compute();
        jacobA.Reset();
        jacobA.SetJendActive();
        switch ( WhichMethod ) {
            case JACOB_TRANS:
                jacobA.SetCurrentMode(JACOB_JacobianTranspose);
                fprintf(stdout,"Testing Jacobian Transpose method.\n");
                break;
            case DLS:
                jacobA.SetCurrentMode(JACOB_DLS);
                jacobA.SetDampingDLS( 1.0 );
                fprintf( stdout, "Testing Damped Least Squares convergence.\n");
                break;
            case SDLS:
                jacobA.SetCurrentMode(JACOB_SDLS);
                fprintf( stdout, "Testing Selectively Damped Least Squares convergence.\n");
                break;
        }
        UpdateTargets(time);
        long i = 0;
        while ( true ) {
            i++;
            jacobA.ComputeJacobian();
            jacobA.CalcDeltaThetas();
            jacobA.UpdateThetas();                                                  // Apply the change in the theta values
            double totalError = jacobA.UpdateErrorArray();
            const VectorRn& err = jacobA.GetErrorArray();
            jacobA.UpdatedSClampValue();            // Only relevant for SDLS or DLS with clamping (not needed for usual DLS)
                fprintf(stdout, "Iteration %2ld: total error = %7lf, (%6lf, %6lf, %6lf, %6lf).\n", i, totalError,
                        err[0], err[1], err[2], err[3]);
                char c = fgetc(stdin);
                if ( c=='.' ) {
                    fprintf(stdout, "\n");
                    break;
                }
                if ( c=='x' ) {
                    return;
                }
        }
    }
}

void RunTestB()
{
    WhichShape = DBLYSHAPE;         // Need to do to update the targets correctly
    Node* nodesA[MAX_NUM_NODE];
    Tree treeA;
    BuildTreeDoubleYShape( nodesA, treeA );
    Jacobian jacobA( &treeA );
    treeA.Init();
    treeA.Compute();
    jacobA.Reset();
    jacobA.SetJendActive();
    jacobA.SetCurrentMode(JACOB_DLS);
    //jacobA.SetDampingDLS( 1.0 );
    
    Node* nodesB[MAX_NUM_NODE];
    Tree treeB;
    BuildTreeDoubleYShape( nodesB, treeB );
    Jacobian jacobB( &treeB );
    treeB.Init();
    treeB.Compute();
    jacobB.Reset();
    jacobB.SetJendActive();
    jacobB.SetCurrentMode(JACOB_SDLS);
    //jacobB.SetDampingDLS( 1.0 );
    
    double time = 0.0;
    long j;
    for ( j=0; j<400; j++) {
        time += Tstep;
        UpdateTargets(time);
        jacobA.ComputeJacobian();
        jacobA.CalcDeltaThetas();
        jacobA.UpdateThetas();                                                  // Apply the change in the theta values
        jacobA.UpdateErrorArray();
        jacobA.UpdatedSClampValue();            // Only relevant for SDLS or DLS with clamping (not needed for usual DLS)
                jacobB.ComputeJacobian();
                jacobB.CalcDeltaThetas();
                jacobB.UpdateThetas();                                                  // Apply the change in the theta values
                jacobB.UpdateErrorArray();
                jacobB.UpdatedSClampValue();            // Only relevant for SDLS or DLS with clamping (not needed for usual DLS)
    }
    double totalErrorA = 0.0;
    double totalErrorB = 0.0;
    double relErrorA, relErrorB;
    double netRelErrorA = 0.0;
    double netRelErrorB = 0.0;
    long win1[5] = {0,0,0,0,0};
    long win2[5] = {0,0,0,0,0};
    int b1, b2, ties;
    int NumTests = 2000;
    double fNumTestsCent = 0.01*NumTests;
    for ( j=0; j<NumTests; j++) {
        time += Tstep;
        UpdateTargets(time);
        jacobA.ComputeJacobian();
        jacobA.CalcDeltaThetas();
        jacobA.UpdateThetas();                                                  // Apply the change in the theta values
        totalErrorA += jacobA.UpdateErrorArray();
        jacobA.UpdatedSClampValue();            // Only relevant for SDLS or DLS with clamping (not needed for usual DLS)
                jacobB.ComputeJacobian();
                jacobB.CalcDeltaThetas();
                jacobB.UpdateThetas();                                                  // Apply the change in the theta values
                totalErrorB += jacobB.UpdateErrorArray();
                jacobB.UpdatedSClampValue();            // Only relevant for SDLS or DLS with clamping (not needed for usual DLS)
                Jacobian::CompareErrors( jacobA, jacobB, &relErrorA, &relErrorB );
                netRelErrorA += relErrorA;
                netRelErrorB += relErrorB;
                Jacobian::CountErrors( jacobA, jacobB, &b1, &b2, &ties );
                win1[b1]++;
                win2[b2]++;
    }
    
    fprintf(stdout, "DLS:  Total error = %8lf.\n", totalErrorA );
    fprintf(stdout, "SDLS: Total error = %8lf.\n", totalErrorB );
    fprintf(stdout, "DLS:  Relative error = %8lf.\n", netRelErrorA );
    fprintf(stdout, "SDLS: Relative error = %8lf.\n", netRelErrorB );
    fprintf(stdout, "DLS:  Number wins  %4.1f%% 0's, %4.1f%% 1's, %4.1f%% 2's, %4.1f%% 3s, %4.1f%% 4's\n",
            win1[0]/fNumTestsCent, win1[1]/fNumTestsCent, win1[2]/fNumTestsCent, win1[3]/fNumTestsCent, win1[4]/fNumTestsCent);
    fprintf(stdout, "SDLS: Number wins  %4.1f%% 0's, %4.1f%% 1's, %4.1f%% 2's, %4.1f%% 3s, %4.1f%% 4's\n",
            win2[0]/fNumTestsCent, win2[1]/fNumTestsCent, win2[2]/fNumTestsCent, win2[3]/fNumTestsCent, win2[4]/fNumTestsCent);
}

void RunTestA() 
{
    Node* nodesA[MAX_NUM_NODE];
    Tree treeA;
    //BuildTreeDoubleYShape( nodesA, treeA );
    BuildTreeYShape( nodesA, treeA );
    Jacobian jacobA( &treeA );
    
    // Loop over different damping factors
    double startDamp = 0.3;
    double stepDamp = 0.01;
    int numDamp = 30;
    for ( long i=0; i<numDamp; i++ ) {
        treeA.Init();
        treeA.Compute();
        jacobA.Reset();
        jacobA.SetJendActive();
        jacobA.SetCurrentMode(JACOB_DLS);
        double thisDamping = startDamp + i*stepDamp;
        jacobA.SetDampingDLS( thisDamping );
        double time = 0.0;
        long j;
        for ( j=0; j<20; j++) {
            time += Tstep;
            UpdateTargets(time);
            jacobA.ComputeJacobian();
            jacobA.CalcDeltaThetas();
            jacobA.UpdateThetas();                                                  // Apply the change in the theta values
            jacobA.UpdateErrorArray();
            jacobA.UpdatedSClampValue();            // Only relevant for SDLS, but no harm down here
        }
        double totalError = 0.0;
        for ( j=0; j<200; j++) {
            time += Tstep;
            UpdateTargets(time);
            jacobA.ComputeJacobian();
            jacobA.CalcDeltaThetas();
            jacobA.UpdateThetas();                                                  // Apply the change in the theta values
            totalError += jacobA.UpdateErrorArray();
            jacobA.UpdatedSClampValue();            // Only relevant for SDLS, but no harm down here
        }
        fprintf(stdout," Damping = %7.4lf: total error = %lf.\n", thisDamping, totalError );
        
    }
}

