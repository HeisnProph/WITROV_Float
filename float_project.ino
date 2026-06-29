#include "float_webserver.h" // 包含了所有函数的声明 all function declarations

// 要额外定义两个头文件和cpp文件是因为arduino的原始文件是以.ino结尾的, 处理web_content文本信息(string)时, 编译器会优化掉很多空格和符号, 导致网页错误
// 需要让运行文件为cpp 把web_content include到cpp文件, 保证运行文件是cpp类型, 编译器才能正常工作. 

// The for needed two header and Cpp files is that orginal Arduino project is ended by 'ino' and its compiler will optimize many spaces and symboles when dealing with string variables which exact the web content is. 
// So need .h file to store the web content and include it in .cpp file, the cpp compiler works fine with those string web contents

void setup() {
  setup_float(); // 声明declaration in 在float_webserver.h , realization in 实现在cpp 
}

void loop() {
  loop_float(); // 声明在float_webserver.h 实现在cpp
}