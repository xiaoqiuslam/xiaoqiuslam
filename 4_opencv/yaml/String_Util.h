#ifndef STRING_UTIL_H
#define STRING_UTIL_H

#include <iostream>
#include <string>
#include <vector>

using namespace std;

std::vector<std::string> getFilesFromJson(const std::string& labelFile, int maxnum);
std::string file2str(const std::string& file);
bool getFilesFromDectory(const string& srcDir, vector<string>& filesPath, bool recursive = true);
int getFilesFromDir(const std::string & srcDir, const std::string & srcSubDir, std::vector<std::string> & filesPath);
bool split(const std::string& str, std::vector< std::string >& ret_, std::string sep);
std::string replace_all_distinct(const std::string&  str, const std::string&   old_value, const   std::string& new_value);
bool getFilenameAndSubffix(const string& imagePath, string& filename, string& filesuffix);
bool getDirNameSubffix(const string& imagePath, string& directory, string& filename, string& filesuffix);

string readIni(string key);


class Roiinfo {
public:
	static std::string Roiinfo::X;
	static std::string Roiinfo::Y;
	static std::string Roiinfo::W;
	static std::string Roiinfo::H;

private: // memory unit
	int x_;
	int y_;
	int w_;
	int h_;


public:  //setter and getter
	 Roiinfo(int x, int y, int w, int h)
	{
		x_ = x;
		y_ = y;
		w_ = w;
		h_ = h;
	}
	int x()
	{
		return x_;
	}
	int y()
	{
		return y_;
	}
	int w()
	{
		return w_;
	}
	int h()
	{
		return h_;
	}

};

class Picinfo
{
public:
	static std::string Picinfo::ROOTNAME;
	static std::string Picinfo::FILENAME;
	static std::string Picinfo::WIDTH;
	static std::string Picinfo::HEIGHT;
	static std::string Picinfo::TYPE;
	static std::string Picinfo::ROINUM;
	static std::string Picinfo::ROIINFO;
private:
	std::string filename_;
	int width_;
	int height_;
	std::string type_;
	int roinum_;
	std::vector<Roiinfo> vroiinfo_;

public:
	Picinfo(std::string filename, int width, int height, std::string type, int roinum, std::vector<Roiinfo> vroiinfo)
	{
		filename_ = filename;
		width_ = width;
		height_ = height;
		type_ = type;
		roinum_ = roinum;
		vroiinfo_ = vroiinfo;
	}

	std::string filename()
	{
		return filename_;
	}
	int width()
	{
		return width_;
	}
	int height()
	{
		return height_;
	}
	std::string type()
	{
		return type_;
	}
	int roinum()
	{
		return roinum_;
	}
	std::vector<Roiinfo> vroiinfo()
	{
		return vroiinfo_;
	}

};

class json_parser
{
//private:
//	static std::string ROOTNAME;
public:
	static std::string generate(const std::vector<Picinfo>& vpicinfo);
	static std::string generatePicinfo(Picinfo& picinfo);

	static bool parse(const std::string& s, std::vector<Picinfo>& vpicinfo);
	static bool parseJsonfile(const std::string& file, std::vector<Picinfo>& vpicinfo);
public:
	static void tester();
};

#endif // !STRING_UTIL_H




