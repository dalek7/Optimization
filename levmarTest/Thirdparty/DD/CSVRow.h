#ifndef __CVSROW_H__
#define __CVSROW_H__
// based on http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


class CSVRow
{
 public:
        std::string const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str,line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream,cell,' '))
            {
                m_data.push_back(cell);
            }
        }


    private:
        std::vector<std::string>    m_data;
};

inline std::istream& operator>>(std::istream& str,CSVRow& data)
{
    data.readNextRow(str);
    return str;
}


/*
// File proc
			string line;
			ifstream myfile (fnsift);
			if (myfile.is_open())
			{
				while ( getline (myfile,line) )
				{

					;//cout << line << '\n';
				}
			myfile.close();
			}


*/


#endif
