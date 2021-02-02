#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>

/*######################################*/
void write_vector_to_file(const std::string s, std::string filename)
{
    std::ofstream ofs(filename, std::ios::app | std::ofstream::binary);
    for(size_t i = 0; i < s.size()-1; i++)

        ofs << s[i];

    ofs << s[s.size()-1]<< std::endl;
}

void printArray(std::vector<double> input)
{
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

int main()
{
    // TO OUPUT TO FILE WITHOUT ",{}"
//    std::string s;
//    std::string chars =",{}";

//    std::ifstream file_in("/home/anders/Hand-eye-Calibration/Robot_control/workcell/test.txt");
//   // if (!file_in) {/*error*/}
//    while(std::getline(file_in,s))
//    {
//       // std::cout << s;
//        for(char c: chars)
//        {
//            s.erase(std::remove(s.begin(), s.end(), c), s.end());

//        }
//        std::cout << s<<std::endl;
//        write_vector_to_file(s,"/home/anders/Hand-eye-Calibration/Robot_control/workcell/test_TCP.txt");


//    }
//    file_in.close();
// TO READ THE DATA INTO VECTORS
    std::vector<std::vector<double>> v;
       std::ifstream in( "/home/anders/Hand-eye-Calibration/Robot_control/workcell/test_TCP.txt" );
       std::string record;

       while ( std::getline( in, record ) )
       {
           std::istringstream is( record );
           std::vector<double> row( ( std::istream_iterator<double>( is ) ),
                                    std::istream_iterator<double>() );
           v.push_back( row );
       }

       for ( const auto &row : v )
       {
           for ( double x : row ) std::cout << x << ' ';
           std::cout << std::endl;
       }

   printArray(v[0]);

}
