#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
int main(){setvbuf(stdout,nullptr,_IONBF,0);return CollisionBudgetTest()?0:1;}
