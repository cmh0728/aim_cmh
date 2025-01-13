#include <iostream> 
#include <vector>
using namespace std;

int main()
{
    cin.tie(NULL);
    ios_base::sync_with_stdio(false);

	int n, m ;
	cin >> n >> m ;

    vector<int> v1(n); // 크기 n으로 초기화 

	for (int x = 0 ; x < m ; x++)
	{
		int i , j , k ;
		cin >> i >> j >> k ;
		
		for (int num = i ; num <= j ; num ++)
		{
			// cout << num << "\n" ;
			v1[num-1] = k ;
		}
	}
	
	for (int v : v1)
	{
		cout << v << " " ;
	}
    return 0;
}
