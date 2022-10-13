//
// Created by yoonsikjung on 2022/10/05.
//

#ifndef BB_NODE_H
#define BB_NODE_H

#include <vector>
#include "gurobi_c++.h"

using namespace std;

class Node{
private:
    bool isPruned;
    double LB;
    vector<double> solution;
    Node* parent;
    Node* left;
    Node* right;
    GRBModel* model = nullptr;
    int nVar;
    int status;

public:

    Node(GRBModel* m, double LB, int nVar) : isPruned(false), parent(nullptr), left(nullptr), right(nullptr){
        model = m;
        this->LB = LB;
        this->nVar = nVar;
    }

    Node(GRBModel* m, Node* parent ,double LB, int nVar): isPruned(false), left(nullptr), right(nullptr){
        model = m;
        this->LB = LB;
        this->nVar = nVar;
        this->parent = parent;
    }

//    ~Node(){
////        auto vars = model->getVars();
////        delete[] vars;
//        delete this->model;
//    }

    void setModel(GRBModel* m){
        model = m;
    }

    void setLeft(Node* node){
        left = node;
    }

    void setRight(Node* node){
        right = node;
    }

    GRBModel* getModel(){
        return model;
    }

    double getLPObj(){
        return this->model->get(GRB_DoubleAttr_ObjVal);
    }

    double getLB(){
        return LB;
    }

    vector<double> getSolution(){ // pointer로 수정 필요
        return solution;
    }

    void solve(){
        try{
            model->optimize();
            LB = model->get(GRB_DoubleAttr_ObjVal);
            auto vars = model->getVars();
            for(int i = 0; i < nVar; i++){
                double x = vars[i].get(GRB_DoubleAttr_X);
                solution.push_back(x);
            }
            int optimstatus = model->get(GRB_IntAttr_Status);
            status = optimstatus;
        }
        catch(GRBException e){
            cout << e.getMessage() << endl;
        }

    };

    bool isIntegerSolution(){
        bool isInteger = true;
        auto vars = model->getVars();

        for(int i = 0; i < nVar; i++){
            double x = vars[i].get(GRB_DoubleAttr_X);
            if(x != (int)x){
                isInteger = false;
            }
        }
        return isInteger;
    }

};

#endif //BB_NODE_H
