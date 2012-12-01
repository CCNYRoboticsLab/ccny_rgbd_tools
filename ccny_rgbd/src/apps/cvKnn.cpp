
/* Command to compile the code: g++ cvKnn.cpp -L /usr/lib -lopencv_core -lopencv_flann -o cvKnn*/

// Include Opencv
#include <opencv/cv.h>

// C++ IO
#include <iostream>

// Namespaces
using namespace cv;
using namespace std;

int numData = 5;
int numQueries = 2;
int numDimensions = 1;
int k = 1;
float Mean = 0.0f;
float Variance = 1.0f;

void printHelpMessage(char *program){
	cout << "****************************************************************************************"<< endl;
	cout << "Usage:: "<<program << "<arg1> <param1> <arg2> <param2> .. <argn> <paramn>" << endl;
	cout << "Where valid arguments are:" << endl;
	cout << "-numData = Number of points in the database(default - 5)" << endl;
	cout << "-numQuries = Number of points in the query set(default - 2)" << endl;
	cout << "-numDimensions = Number of dimensions of the data(default - 1)" << endl;
	cout << "-k = number of nearest neighbors to consider(default - 1)" << endl;
	cout << "-Mean = Mean of the data points(used for generating random data)(default - 0)" << endl;
	cout << "-Variance = Variance of the data points(used for generating random data)(default - 1)" << endl;
	cout << "-help = To print this message" << endl;
	cout << "****************************************************************************************"<< endl;
}

bool parseInput(int argc, char **argv) {
	for(int i = 1 ; i < argc ; i+=2){
		if(strcmp(argv[i],"-help") == 0){ // Help message
			exit(0);
		}
		else if(strcmp(argv[i],"-numData") == 0){ // Parse number of data points
			numData = atoi(argv[i+1]);
			if( numData <= 0){
				cout << " Number of data points need to be positive" << endl;
			}	
		}
		else if(strcmp(argv[i],"-numQueries") == 0){ // Prase number of query points
			numQueries = atoi(argv[i+1]);
			if( numQueries <= 0){
				cout << " Number of query points need to be positive" << endl;
			}	
		}
		else if(strcmp(argv[i],"-numDimensions") == 0){ // Prase number of dimensions
			numDimensions = atoi(argv[i+1]);
			if( numDimensions <= 0){
				cout << " Number of dimensions need to be positive" << endl;
			}	
		}
		else if(strcmp(argv[i],"-k") == 0){ // Parse the number of nearest neighbors
			k = atoi(argv[i+1]);
			if( k <= 0 ){
				cout << " Number of nearest neighbors has to be positive" << endl;
			}	
			if( k > numData ){
				cout << " Number of nearest neighbors has to be less than data size" << endl;
			}	
		}
		else if(strcmp(argv[i],"-mean") == 0){ // Prase the Mean
			Mean = atof(argv[i+1]);
		}
		else if(strcmp(argv[i],"-variance") == 0){ // Prase the Variance
			Variance = atof(argv[i+1]);
		}
	}
}

// Main
int main(int argc, char** argv)
{
	// Print usage
	printHelpMessage(argv[0]);

	// Parse input
	parseInput(argc, argv);

	// Create the data
	Mat features(numData,numDimensions,CV_32F), query(numQueries,numDimensions,CV_32F);
	randu(features, Scalar::all(Mean), Scalar::all(Variance));
	randu(query, Scalar::all(Mean), Scalar::all(Variance));

	// Print generated data
	cout << "Input::" << endl;
	for(int row = 0 ; row < features.rows ; row++){
		for(int col = 0 ; col < features.cols ; col++){
			cout << features.at<float>(row,col) <<"\t";
		}
		cout << endl;
	}
	cout << "Query::" << endl;
	for(int row = 0 ; row < query.rows ; row++){
		for(int col = 0 ; col < query.cols ; col++){
			cout << query.at<float>(row,col) <<"\t";
		}
		cout << endl;
	}

	// KdTree with 5 random trees
	cv::flann::KDTreeIndexParams indexParams(5);

	// You can also use LinearIndex
	//cv::flann::LinearIndexParams indexParams;

	// Create the Index
	cv::flann::Index kdtree(features, indexParams);

	// Perform single search for mean
	cout << "Performing single search to find closest data point to mean:" << endl;
	vector<float> singleQuery;
	vector<int> index(1);
	vector<float> dist(1);

	// Searching for the Mean
	for(int i = 0 ; i < numDimensions ;i++)
		singleQuery.push_back(Mean);

	// Invoke the function
	kdtree.knnSearch(singleQuery, index, dist, 1, cv::flann::SearchParams(64));

	// Print single search results
	cout << "(index,dist):" << index[0] << "," << dist[0]<< endl;

	// Batch: Call knnSearch
	cout << "Batch search:"<< endl;
	Mat indices;//(numQueries, k, CV_32S);
	Mat dists;//(numQueries, k, CV_32F);

	// Invoke the function
	kdtree.knnSearch(query, indices, dists, k, cv::flann::SearchParams(64));

	cout << indices.rows << "\t" << indices.cols << endl;
	cout << dists.rows << "\t" << dists.cols << endl;

	// Print batch results
	cout << "Output::"<< endl;
	for(int row = 0 ; row < indices.rows ; row++){
		cout << "(index,dist):"; 
		for(int col = 0 ; col < indices.cols ; col++){
			cout << "(" << indices.at<int>(row,col) << "," << dists.at<float>(row,col) << ")" << "\t";
		}
		cout << endl;
	}
	return 0;
}
