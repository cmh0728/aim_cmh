#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int main()
{
	cin.tie(NULL);
	ios_base::sync_with_stdio(false);

	vector<int> v1(30);

	//기본값 할당
	for(int i = 1 ; i <= 30 ; i++)
	{
		v1[i-1] = 0 ;
	}

	for(int i =1 ; i<=28 ; i ++)
	{
		int num ;
		cin >> num ;
		v1[num-1] = 1;
	}

	//출력
	for (int i =1 ;i<=30;i++)
	{
		if (v1[i-1] == 0 )
		{
			cout << i << "\n" ;
		}
	}

	return 0;
}