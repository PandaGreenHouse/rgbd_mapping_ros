#ifndef PLUGIN_H
#define PLUGIN_H
extern "C"{
void initplugin(const char *thepath);
void getpose(float *pose);
void getNewData(int *imgBuf, float *quadBuf, float *boxBuf, int *numVertices, float *poseBuf);
int getwidth();
int getheight();
}
#endif // PLUGIN_H
