#include "IpIpoptApplication.hpp"
#include "example_nlp.hpp"
#include <iostream>


int main(int /*argv*/, char** /*argc*/){
    
    //New instance of the NLP
    Ipopt::SmartPtr<TNLP> mynlp = new HS071_NLP();

    //New instance of IPOPT application
    //Factory allows to compile with IPOPT windows dll
    Ipopt::SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    //set values according to the needs of your problem:
    app->Options()->SetNumericValue("tol", 3.82e-6);
    app->Options()->SetStringValue("mu_strategy", "adaptative");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("option_file_name", "example.opt");

    //Initialize the application and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if(  status != Ipopt::Solve_Succeeded ){
        std::cout<< std::endl << std::endl << "*** Error During Initialization" << std::endl;
        return (int) status;
    }
    
    //Call to solve the problem
    status = app->OptimizeTNLP(mynlp);
    
    //process the result to the call
    if (status == Ipopt::Solve_Succeeded){
        std::cout << std::endl << std::endl << "*** Problem Solved Successfully" << std::endl;
    }else{
        std::cout << std::endl << std::endl << "*** Problem Failed" << std::endl;
    }

    // As the SmartPtrs go out of scope, the reference count
    // will be decremented and the objects will automatically
    // be deleted.

    return (int) status;
}

