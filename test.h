//
// Created by yoonsikjung on 2022/10/12.
//

#ifndef BB_TEST_H
#define BB_TEST_H
#include <string>
#include "gurobi_c++.h"
#include "branchandbound.h"
#include "utils.h"

using namespace std;

void mip1(){
    try {
    // Create an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", "mip1.log");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    // Create variables
    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

    // Set objective: maximize x + y + 2 z
    model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

    // Add constraint: x + 2 y + 3 z <= 4
    model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

    // Add constraint: x + y >= 1
    model.addConstr(x + y >= 1, "c1");

    // Optimize model
    model.optimize();

    cout << x.get(GRB_StringAttr_VarName) << " "
         << x.get(GRB_DoubleAttr_X) << endl;
    cout << y.get(GRB_StringAttr_VarName) << " "
         << y.get(GRB_DoubleAttr_X) << endl;
    cout << z.get(GRB_StringAttr_VarName) << " "
         << z.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

} catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
} catch(...) {
    cout << "Exception during optimization" << endl;
}

};


void testModel(){
    vector<double> c = {8, 11, 6, 4};
    vector<vector<double>> A;
    A.push_back({5,7,4,3});
    vector<double> b = {14};
    // all x are binary
}

void testModelFile(){
    try{
        GRBEnv env = GRBEnv();
        GRBModel m = GRBModel(env, "../test.lp");
        m.set(GRB_IntAttr_ModelSense, 1); // change to minimization problem
        GRBVar* vars = m.getVars();
        for(int i =0; i < 4; i++){
            vars[i].set(GRB_CharAttr_VType, 'c');
            vars[i].set(GRB_DoubleAttr_Obj, -vars[i].get(GRB_DoubleAttr_Obj));
        }

        m.optimize();
//        cout << vars[0].get(GRB_DoubleAttr_Obj) << endl;
    }
    catch(GRBException e) {
        cout << e.getErrorCode() << " "<< e.getMessage() << endl;
    }
}

void BNBTest(string fname){
    BranchAndBound bnb = BranchAndBound(fname);
//    bnb.subproblemSelection();
    clock_t start, end;
    double res;
    start = clock();
    bnb.run();
    end = clock();
    res = (double) (end - start) / CLOCKS_PER_SEC;
    cout << res << endl;

    clock_t startg, endg;
    startg = clock();
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env, fname);

    model.optimize();
    endg = clock();
    res = (double) (endg - startg) / CLOCKS_PER_SEC;
    cout << res << endl;

}

void exportSCModel(string fname){
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    vector<vector<double>> binMat;
    readData<bool>(binMat, fname, ' ');
    int nCols = binMat[0].size();
    int nRows = binMat.size();

    vector<GRBVar> vars;
    vars.resize(nCols);

    for(int i = 0; i < nCols; i++){
        vars[i] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY, "X_"+to_string(i));
    }

    for(int i = 0; i < nRows; i++){
        GRBLinExpr expr = 0;
        double sum = 0;
        for(int j = 0; j < nCols; j++){
            sum += binMat[i][j];
            expr += vars[j] * binMat[i][j];
        }
        if(sum == 0)
            continue;
        model.addConstr(expr >= 1, "row_"+ to_string(i));
    }
    // minimize
    model.set(GRB_IntAttr_ModelSense, 1);

    model.optimize();
    model.write(fname+".lp");

}

#endif //BB_TEST_H
