#ifndef SLAMEXPRESS_LOADER_INDEXER
#define SLAMEXPRESS_LOADER_INDEXER

#include <boost/filesystem.hpp>

/* Indexer loads an index filename from disk, and return its tokens.
 * Each line in the filename is broken into multiple tokens. 
 */
class Indexer {
protected:
    // convert a string to a series of tokens, seperated by delimiters
    void tokenize (const std::string &str, std::vector<std::string> &tokens, std::string delimiters = " ")  {
        tokens.clear();

        std::string::size_type lastPos    = str.find_first_not_of (delimiters, 0);
        std::string::size_type pos        = str.find_first_of     (delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos) {
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            lastPos = str.find_first_not_of(delimiters, pos);
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    std::ifstream _index_file_stream; //the index 
    std::vector<std::string> tokens;
public:
    std::string _directory; //full path to parent directory (dataset main dir)
    std::string _dataset_name; //name of the parent dir

    bool load_index(std::string index_file) {
        boost::filesystem::path index_file_path(index_file.c_str());
        _directory = index_file_path.parent_path().string();
        _dataset_name = index_file_path.parent_path().filename().string();

        _index_file_stream.open(index_file);
        if(_index_file_stream.fail()) {
            std::cerr << "[Indexer] " << "Error loading index " << index_file << std::endl;
            return false;
        }
        return true;
    }
    
    // retrieve the dataset image size 
    std::pair<int,int> get_dataset_resolution() {
        load_data();
        cv::Mat temp = cv::imread(get_current_filename());
    
        _index_file_stream.clear();
        _index_file_stream.seekg(0);

        return std::make_pair(temp.cols, temp.rows);
    }

    // retrieve the dataset image type
    int get_dataset_image_type() {
        load_data();
        cv::Mat temp = cv::imread(get_current_filename());
    
        _index_file_stream.clear();
        _index_file_stream.seekg(0);

        return temp.type();
    }

    // load one line of tokens from the index
    bool load_data() {
        std::string currentLine;
        tokens.clear();

        if(_index_file_stream.eof()) return false;

        getline(_index_file_stream, currentLine);
        tokenize(currentLine, tokens);

        if (tokens.size() == 0)
            return false;

        return true;
    }

    // get the first token as double (timestamp)
    double get_current_timestamp() {
        return stod(tokens[0]);
    }

    // return a string of the full path of the token (appended by the directory)
    std::string get_current_filename(unsigned int token_index=1) {
        if(token_index>=tokens.size()) std::cerr << "[Indexer] " << "Wrong index passed\n";
        std::string fileLoc = _directory;
        fileLoc.append("/");
        fileLoc.append(tokens[token_index]);
        return fileLoc;
    }

    // cout the tokens of the current line
    void output_current_tokens() {
        std::string stamp = tokens[0];
        std::cout << "[Indexer]" << "[" << stamp << "] ";
        for(unsigned int i = 1; i < tokens.size(); i++ ) {
            std::string fileLoc = _directory;
            fileLoc.append("/");
            fileLoc.append(tokens[i]);

            std::cout << fileLoc.c_str() << " ";
        }
        std::cout << std::endl;
    }
};

#endif