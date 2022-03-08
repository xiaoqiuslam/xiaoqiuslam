#include "String_Util.h"
#include <io.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <sstream> 
#include <vector>

using namespace std;

std::string Picinfo::ROOTNAME = "vpicinfo";
std::string Picinfo::FILENAME = "filename";
std::string Picinfo::WIDTH = "width";
std::string Picinfo::HEIGHT = "height";
std::string Picinfo::TYPE = "type";
std::string Picinfo::ROINUM = "roinum";
std::string Picinfo::ROIINFO = "roiparameter";

std::string Roiinfo::X = "x";
std::string Roiinfo::Y = "y";
std::string Roiinfo::W = "w";
std::string Roiinfo::H = "h";


std::string file2str(const std::string& file){
	ifstream fin(file);
	ostringstream sin;
	sin << fin.rdbuf();
	std::string s = sin.str();
	fin.close();
	fin.clear();

	return s;
}


bool split(const string& str, vector<string>& ret_, string sep) {
	if (str.empty()) {
		return 0;
	}
	string tmp;
	string::size_type pos_begin = str.find_first_not_of(sep);
	string::size_type comma_pos = 0;
	while (pos_begin != string::npos) {
		comma_pos = str.find(sep, pos_begin);
		if (comma_pos != string::npos) {
			tmp = str.substr(pos_begin, comma_pos - pos_begin);
			pos_begin = comma_pos + sep.length();
		} else {
			tmp = str.substr(pos_begin);
			pos_begin = comma_pos;
		}
		if (!tmp.empty()) {
			ret_.push_back(tmp);
			tmp.clear();
		}
	}
	return true;
}


std::string replace_all_distinct(const std::string&  str, const std::string&   old_value, const   std::string& new_value){
//string&  replace_all_distinct(string&   str, const   string&   old_value, const   string&   new_value)
//{
	//ostringstream oss(str);
	ostringstream oss;
	oss << str;
	std::string srcStr = oss.str();

	for (string::size_type pos(0); pos != string::npos; pos += new_value.length())   {
		if ((pos = srcStr.find(old_value, pos)) != string::npos)
			 srcStr = srcStr.replace(pos, old_value.length(), new_value);
		else   break;
	}
	return srcStr ;
}


int getFilesFromDir(const string& srcDir, const string& srcSubDir, vector<string>& filesPath)
{
	if (srcDir.empty() || srcSubDir.empty())
		return 0;

	string absDir = "";
	if (srcSubDir.length() - srcDir.length()>2)
		absDir = srcSubDir.substr(srcDir.length()+1, (srcSubDir.length() - srcDir.length()));

	long hFile = 0;

	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(srcSubDir).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//�����Ŀ¼,����֮  
			//�������,�����б�  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFilesFromDir(srcDir, p.assign(srcSubDir).append("/").append(fileinfo.name), filesPath);
			}
			else
			{
				//filesPath.push_back(p.assign(srcSubDir).append("\\").append(fileinfo.name));
				string path;
				if (!absDir.empty())
					path = p.assign(absDir).append("/").append(fileinfo.name);
				else
					path = fileinfo.name;

				if (path.length()>4 && (
					path.substr(path.length() - 4, 4).compare(".jpg") == 0 ||
					path.substr(path.length() - 4, 4).compare(".bmp") == 0 ||
					path.substr(path.length() - 4, 4).compare(".png") == 0 ||
					path.substr(path.length() - 5, 5).compare(".h264") == 0 ||
					path.substr(path.length() - 4, 4).compare(".mp4") == 0
					))
					filesPath.push_back(path);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


bool scanDir(const string& srcDir, const string& srcSubDir, vector<string>& filesPath, bool recursive = true) {

	if (recursive){
		if (srcDir.empty() || srcSubDir.empty())
			return false;

		string absDir = "";
		if (srcSubDir.length() - srcDir.length() > 2)
			absDir = srcSubDir.substr(srcDir.length() + 1, (srcSubDir.length() - srcDir.length())); // +1 becasue of append("/")

		long hFile = 0;
		struct _finddata_t fileinfo;
		string p;
		if ((hFile = _findfirst(p.assign(srcSubDir).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//�����Ŀ¼,����֮  
				if ((fileinfo.attrib &  _A_SUBDIR)) { 
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
						scanDir(srcDir, p.assign(srcSubDir).append("/").append(fileinfo.name), filesPath, true);
				} else {
				//�������,�����б�  

					//filesPath.push_back(p.assign(srcSubDir).append("\\").append(fileinfo.name));
					string path;
					if (!absDir.empty())
						path = p.assign(absDir).append("/").append(fileinfo.name);
					else
						path = fileinfo.name;

					if (path.length() > 4 && (
						path.substr(path.length() - 4, 4).compare(".jpg") == 0 ||
						path.substr(path.length() - 4, 4).compare(".bmp") == 0 ||
						path.substr(path.length() - 4, 4).compare(".png") == 0 ||
						path.substr(path.length() - 5, 5).compare(".h264") == 0 ||
						path.substr(path.length() - 4, 4).compare(".mp4") == 0
						))
						filesPath.push_back(path);
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	} else{
		if (srcDir.empty() || srcSubDir.empty())
			return false;

		long hFile = 0;
		struct _finddata_t fileinfo;
		string p;
		if ((hFile = _findfirst(p.assign(srcDir).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//���ļ�,����Ŀ¼
				if (!(fileinfo.attrib &  _A_SUBDIR)) { 
				
					string path = fileinfo.name;

					if (path.length() > 4 && (
						path.substr(path.length() - 4, 4).compare(".jpg") == 0 ||
						path.substr(path.length() - 4, 4).compare(".bmp") == 0 ||
						path.substr(path.length() - 4, 4).compare(".png") == 0 ||
						path.substr(path.length() - 5, 5).compare(".h264") == 0 ||
						path.substr(path.length() - 4, 4).compare(".mp4") == 0
						))
						filesPath.push_back(path);
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}

	}
}


bool getFilesFromDectory(const string& srcDir, vector<string>& filesPath, bool recursive) {
	return  scanDir(srcDir, srcDir, filesPath, recursive);
}


bool getFilenameAndSubffix(const string& imagePath, string& filename, string& filesuffix){
		int dir = imagePath.find_last_of("/");
		if (dir == -1){
			dir = imagePath.find_last_of("\\");
		}
		int dot = imagePath.find_last_of(".");
		filename = imagePath.substr(dir + 1, dot - (dir + 1));
		filesuffix = imagePath.substr(dot);
		return true;
}


bool getDirNameSubffix(const string& imagePath,string& directory, string& filename, string& filesuffix){
		int dir = imagePath.find_last_of("/");
		if (dir == -1){
			dir = imagePath.find_last_of("\\");
		}
		int dot = imagePath.find_last_of(".");
		directory = imagePath.substr(0, dir+1);
		filename = imagePath.substr(dir + 1, dot - (dir + 1));
		filesuffix = imagePath.substr(dot+1);
		return true;
}
