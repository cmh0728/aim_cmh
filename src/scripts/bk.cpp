# include <iostream>
# include <algorithm>
# include <vector>

using namespace std;

int main()
{
	cin.tie(NULL);
	ios_base::sync_with_stdio(false);

	int n , k ;
	cin >> n >> k ;

	vector<int> v1(n);
	for (int i  = 0 ;  i < n ; i ++)
	{
		cin >> v1[i];
	}

	sort(v1.begin(),v1.end(),greater<int>());

	//디버깅용
	// for (int i = 0 ; i < n ; i ++)
	// {
	// 	cout << v1[i]<< " ";
	// }

	cout << v1[k-1];
	return 0;
}
