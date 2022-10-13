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

class BranchAndBound{
private:
    double LB;
    double UB;
    vector<Node*> solved;
    vector<Node*> unsolved;// unsolved를 binary tree형태로 구성하면 minimum값 찾는 것이 빨라질듯? tie까지 고려할 수 있는 자료구조 고려해볼것
    Node* bestNode;
    Node* root;
    int nVar;
    int nNode = 0;
    bool terminated = false;
    vector<int> integerVarIdx;


public:
    BranchAndBound(){

    }

    BranchAndBound(string fname): bestNode(nullptr), terminated(false){

        LB = numeric_limits<double>::infinity();
        UB = numeric_limits<double>::infinity();

        try{
            GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);
            GRBModel* m = new GRBModel(env, fname);
            m->set(GRB_IntAttr_ModelSense, 1); // change to minimization problem
            GRBVar* vars = m->getVars();
            for(int i =0; i < m->get(GRB_IntAttr_NumVars); i++){ // LP relexation & change objective sense to negative
                vars[i].set(GRB_CharAttr_VType, 'c');
                if(m->get(GRB_IntAttr_ModelSense) == -1){
                    vars[i].set(GRB_DoubleAttr_Obj, -vars[i].get(GRB_DoubleAttr_Obj));
                }
                else{
                    vars[i].set(GRB_DoubleAttr_Obj, vars[i].get(GRB_DoubleAttr_Obj));
                }

            }
            nVar = m->get(GRB_IntAttr_NumVars);
            Node* root = new Node(m, numeric_limits<double>::infinity(), nVar);
            this->root = root;
            unsolved.push_back(root);
        }
        catch(GRBException e) {
            cout << e.getErrorCode() << " "<< e.getMessage() << endl;
        }
    }

    void subproblemSelection(){
        if(this->unsolved.size() == 0){
            cout << "terminate B&B" << endl;
            terminated = true;
        }
        else{
            auto minNodes = min_element(unsolved.begin(), unsolved.end(), [](auto lhs, auto rhs){
                return lhs->getLB() > rhs->getLB();
            });
            auto minNode = minNodes[0];
            (*minNode).solve();
            int status = minNode->getModel()->get(GRB_IntAttr_Status);
            if(status == GRB_OPTIMAL){
                double lpObj = minNode->getLPObj();
                bool isInteger = minNode->isIntegerSolution();
                solved.push_back(minNode);
                unsolved.erase(find(unsolved.begin(), unsolved.end(), minNode));
                if(isInteger){
                    // terminate by solving
                    if(lpObj < UB){
                        UB = lpObj;
                        bestNode = minNode;
                        bound();
                    }
                }
                else{
                    if(lpObj < LB){
                        LB = lpObj;
                    }
                    heuristicVariableSelection(minNode);
                }
            }
            else{
                unsolved.erase(find(unsolved.begin(), unsolved.end(), minNode));
            }
        }
    }

    bool checkPruning(Node* node){
        if(node->getLB() > UB){
            return true;
        }
        else
            return false;
    }

    void bound(){
        unsolved.erase(remove_if(unsolved.begin(), unsolved.end(), [this](Node* n){ return n->getLB() > UB;}), unsolved.end());
    }

    void heuristicVariableSelection(Node* node){
        // split into two subproblem and append to unsolved
//        try{

            auto vars = node->getModel()->getVars();
            auto sols = node->getSolution();
            double cutpoint = 0;
            int branchingVarIdx = 0;
            for(int i = 0; i < nVar; i++){
//                double x = vars[i].get(GRB_DoubleAttr_X);
                double x = sols[i];
                if(x != (int)x){
                    cutpoint = x;
                    branchingVarIdx = i;
                    break;
                }
            }
        // generate subproblems

            int cCutpoint = ceil(cutpoint);
            int fCutpoint = floor(cutpoint);

            GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);
            GRBModel* lModel = new GRBModel(*(node->getModel()));
            lModel->addConstr(vars[branchingVarIdx] <= fCutpoint);
            Node* lNode = new Node(lModel, node, node->getLPObj(), nVar);
            nNode++;

            GRBModel* rModel = new GRBModel(*(node->getModel()));
            rModel->addConstr(vars[branchingVarIdx] >= cCutpoint);
            Node* rNode = new Node(rModel, node, node->getLPObj(), nVar);
            nNode++;

            node->setLeft(lNode);
            node->setRight(rNode);

            // add to unsolved
            unsolved.push_back(lNode);
            unsolved.push_back(rNode);
//        }
//        catch(GRBException e){
//            cout << e.getErrorCode()<<" "<< e.getMessage() << endl;
//            cout << "error" << endl;
//        }
//        cout << endl;

    }

    void run(){
        while(!terminated){
            subproblemSelection();
        }
        cout << bestNode->getLPObj() << endl;
        for(auto s : bestNode->getSolution()){
            cout << s << " ";
            cout << endl;
        }
    }

    bool isTerminated(){
        return terminated;
    }
};
#endif //BB_BRANCHANDBOUND_H
