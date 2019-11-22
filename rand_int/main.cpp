#include <iostream>
#include <stdlib.h>
#include <time.h>
using namespace std;

int main()
{
srand((unsigned)time(NULL));
for(int i = 0; i < 100;i++ )
        cout << (rand() % (10+10+1))-10 << endl;
return 0;
}