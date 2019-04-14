#include "iostream"
#include "include/lib_io.h"
#include "include/solution.h"

using namespace std;

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
//        if(argc < 6){
//            std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
//            exit(1);
//        }

//        std::string carPath(argv[1]);
//        std::string roadPath(argv[2]);
//        std::string crossPath(argv[3]);
//        std::string presetAnswerPath(argv[4]);
//        std::string answerPath(argv[5]);

//        std::cout << "carPath is " << carPath << std::endl;
//        std::cout << "roadPath is " << roadPath << std::endl;
//        std::cout << "crossPath is " << crossPath << std::endl;
//        std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
//        std::cout << "answerPath is " << answerPath << std::endl;

    // for debug
    string carPath("../2-map-exam-2/car.txt");
    string roadPath("../2-map-exam-2/road.txt");
    string crossPath("../2-map-exam-2/cross.txt");
    string presetAnswerPath("../2-map-exam-2/presetAnswer.txt");
    string answerPath("../2-map-exam-2/answer.txt");
    // end
	
	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
    std::cout << "presetPath is " << presetAnswerPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;
	
	// TODO:read input filebuf
    Graph g;
    int num_cross = read_crossfile(crossPath,g);
    int num_roads = read_roadfile(roadPath,g);
    int num_cars = read_carfile(carPath,g);
    int num_presetCars = read_presetcarfile(presetAnswerPath,g);
    cout<<"num of cross: "<<num_cross<<endl;
    cout<<"num of roads: "<<num_roads<<endl;
    cout<<"num of cars: "<<num_cars<<endl;
    cout<<"num of preset cars: "<<num_presetCars<<endl;

    // test the answer
//    int num_answer = read_answerfile(answerPath,g);
//    cout<<"num of answers: "<<num_answer<<endl;


    g.loadGraph();
//    g.loadGraphDirect();



    g.printTotalOccupy();
//    g.adjustPlanningTime();

	// TODO:process
    Solution sol;
    int time=sol.processSchdule(g,0);
    cout<<"finish time: "<<time<<endl;


//    int try_num=1;
//    int i=0;
//    vector<int> result;
//    while (i<try_num) {
//        ++i;
//        cout<<"trial: "<<i<<endl;
//        int time=sol.processSchdule(g,0);
//        cout<<"total time: "<<time<<endl;
//        result.push_back(time);
//        sol.adjustSomething(g);
//    }

//    for (int i=0; i<result.size(); ++i) {
//        cout<<"trial: "<<i<<" result: "<<result[i]<<endl;
//    }


    // TODO:write output file
    write_answerfile(answerPath,g);



	
	return 0;
}
