//
// Created by yoonsikjung on 2022/10/06.
//

#ifndef BB_BRANCHANDBOUND_H
#define BB_BRANCHANDBOUND_H

#include <vector>
#include "node.h"
#include "tree.h"
#include <cmath>

using namespace std;

class BranchAndBound {
private:
    double LB;
    double UB;
    vector<Node *> solved;
    vector<Node *> unsolved;// unsolved를 binary tree형태로 구성하면 minimum값 찾는 것이 빨라질듯? tie까지 고려할 수 있는 자료구조 고려해볼것
    Node *bestNode;
    Node *root;
    int nVar;
    int nNode = 0;
    bool terminated = false;
    vector<int> integerVarIdx;
    int solvedNode = 0;
    GRBEnv env = GRBEnv();


public:
    BranchAndBound() {

    }

    BranchAndBound(string fname) : bestNode(nullptr), terminated(false) {

        LB = numeric_limits<double>::infinity();
        UB = numeric_limits<double>::infinity();

        try {
//            GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);
            GRBModel *m = new GRBModel(env, fname);
            m->set(GRB_IntAttr_ModelSense, 1); // change to minimization problem
            GRBVar *vars = m->getVars();
            for (int i = 0;
                 i < m->get(GRB_IntAttr_NumVars); i++) { // LP relexation & change objective sense
                vars[i].set(GRB_CharAttr_VType, 'c');
                if (m->get(GRB_IntAttr_ModelSense) == -1) {
                    vars[i].set(GRB_DoubleAttr_Obj, -vars[i].get(GRB_DoubleAttr_Obj));
                } else {
                    vars[i].set(GRB_DoubleAttr_Obj, vars[i].get(GRB_DoubleAttr_Obj));
                }

            }
            nVar = m->get(GRB_IntAttr_NumVars);
            Node *root = new Node(m, numeric_limits<double>::infinity(), nVar);
            this->root = root;
            unsolved.push_back(root);
        }
        catch (GRBException e) {
            cout << e.getErrorCode() << " " << e.getMessage() << endl;
        }
    }

    void subproblemSelection() {
        if (this->unsolved.size() == 0) {
            cout << "terminate B&B" << endl;
            terminated = true;
        } else {
            auto minNodes = min_element(unsolved.begin(), unsolved.end(), [](auto lhs, auto rhs) {
                return lhs->getLB() < rhs->getLB(); //  > depth first search < best first search
            });
            auto minNode = minNodes[0];
            (*minNode).solve();
            cout << solvedNode << "\t"<< "SELECTED NODE'S LB : "<< minNode->getLB() << "\tLP obj : " << minNode->getLPObj() << endl;
            int status = minNode->getModel()->get(GRB_IntAttr_Status);

            if (status == GRB_OPTIMAL) { // LP Feasible
                double lpObj = minNode->getLPObj();
                bool isInteger = minNode->isIntegerSolution();
                solvedNode++;
//                solved.push_back(minNode);
                unsolved.erase(find(unsolved.begin(), unsolved.end(), minNode));
                if (isInteger) {
                    // terminate by solving
                    if (lpObj < UB) {
                        UB = lpObj;
                        bestNode = minNode;
                        cout << solvedNode << "\t"<< "Incumbent solution Found : "<< UB << endl;
                        bound();
                    }
                } else {
                    if(lpObj > UB){ // terminate by bound
//
                    }
                    else {
//                        heuristicVariableSelection(minNode);
                        pointFiveVariableSelection(minNode);
                    }
                    delete minNode;
                    minNode = nullptr;
                }
            } else { // terminate by infeasibility
                unsolved.erase(find(unsolved.begin(), unsolved.end(), minNode));
                delete minNode;
                minNode = nullptr;
            }
        }
    }

    bool checkPruning(Node *node) {
        if (node->getLB() > UB) {
            return true;
        } else
            return false;
    }

    void bound() {
        unsolved.erase(remove_if(unsolved.begin(), unsolved.end(), [this](Node *n) { return n->getLB() > UB; }),
                       unsolved.end());
    }

    void pointFiveVariableSelection(Node *node, double randomness = 0.1){
        try {

            auto vars = node->getModel()->getVars();
            auto sols = node->getSolution();
            double cutpoint = 0;
            double diff = 9999;
            int branchingVarIdx = 0;
            for (int i = 0; i < nVar; i++) {
                double x = sols[i];
                if (x != (int) x) {
                    if(abs(0.5 - x) < diff){
                        diff = abs(0.5 - x);
                        cutpoint = x;
                        branchingVarIdx = i;
                        if(diff == 0){
                            break;
                        }
                    }
                }
            }

            // generate subproblems (Branching) -> 추후 독립적인 함수로 분할할 것
            int cCutpoint = ceil(cutpoint);
            int fCutpoint = floor(cutpoint);

//            GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);
            GRBModel *lModel = new GRBModel(*(node->getModel()));
//            lModel->addConstr(vars[branchingVarIdx] <= fCutpoint); // for integer variable
            vars = lModel->getVars();
            // fix variable
            vars[branchingVarIdx].set(GRB_DoubleAttr_UB, fCutpoint);
            vars[branchingVarIdx].set(GRB_DoubleAttr_LB, fCutpoint);
            Node *lNode = new Node(lModel, node, node->getLPObj(), nVar);
            nNode++;

            GRBModel *rModel = new GRBModel(*(node->getModel()));
//            rModel->addConstr(vars[branchingVarIdx] >= cCutpoint);
            vars = rModel->getVars();
            // fix variable
            vars[branchingVarIdx].set(GRB_DoubleAttr_UB, cCutpoint);
            vars[branchingVarIdx].set(GRB_DoubleAttr_LB, cCutpoint);
            Node *rNode = new Node(rModel, node, node->getLPObj(), nVar);
            nNode++;

//            node->setLeft(lNode);
//            node->setRight(rNode);

            // add to unsolved
            unsolved.push_back(lNode);
            unsolved.push_back(rNode);
        }
        catch (GRBException e) {
            cout << e.getErrorCode() << " " << e.getMessage() << endl;
            cout << "error" << endl;
        }
//        cout << endl;

    }

    void heuristicVariableSelection(Node *node) {
        // split into two subproblems and append to unsolved
        try {

            auto vars = node->getModel()->getVars();
            auto sols = node->getSolution();
            double cutpoint = 0;
            int branchingVarIdx = 0;
            for (int i = 0; i < nVar; i++) {
                double x = sols[i];
                if (x != (int) x) {
                    cutpoint = x;
                    branchingVarIdx = i;
                    break;
                }
            }

            // generate subproblems (Branching) -> 추후 독립적인 함수로 분할할 것
            int cCutpoint = ceil(cutpoint);
            int fCutpoint = floor(cutpoint);

//            GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);
            GRBModel *lModel = new GRBModel(*(node->getModel()));
//            lModel->addConstr(vars[branchingVarIdx] <= fCutpoint); // for integer variable
            vars = lModel->getVars();
            // fix variable
            vars[branchingVarIdx].set(GRB_DoubleAttr_UB, fCutpoint);
            vars[branchingVarIdx].set(GRB_DoubleAttr_LB, fCutpoint);
            Node *lNode = new Node(lModel, node, node->getLPObj(), nVar);
            nNode++;

            GRBModel *rModel = new GRBModel(*(node->getModel()));
//            rModel->addConstr(vars[branchingVarIdx] >= cCutpoint);
            vars = rModel->getVars();
            // fix variable
            vars[branchingVarIdx].set(GRB_DoubleAttr_UB, cCutpoint);
            vars[branchingVarIdx].set(GRB_DoubleAttr_LB, cCutpoint);
            Node *rNode = new Node(rModel, node, node->getLPObj(), nVar);
            nNode++;

//            node->setLeft(lNode);
//            node->setRight(rNode);

            // add to unsolved
            unsolved.push_back(lNode);
            unsolved.push_back(rNode);
        }
        catch (GRBException e) {
            cout << e.getErrorCode() << " " << e.getMessage() << endl;
            cout << "error" << endl;
        }
//        cout << endl;

    }

    void run() {
        cout << "Iter\tInfo" << endl;
        while (!terminated) {
            subproblemSelection();
            cout << solvedNode << "\t" << " unsolved: " << unsolved.size() << endl;

        }
        cout << bestNode->getLPObj() << endl;
//        for (auto s: bestNode->getSolution()) {
//            cout << s << " ";
////
//        }
        cout << endl;
    }

    bool isTerminated() {
        return terminated;
    }

    vector<double> getSolution(){
        return bestNode->getSolution();
    }
};

#endif //BB_BRANCHANDBOUND_H
