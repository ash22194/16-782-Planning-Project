function [] = command()
    mex plannerCHOMP.cpp -llapacke -lblas -larmadillo
end