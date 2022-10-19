#include "depth_edge.h"
#include "util.h"

void jump_edge(cv::Mat &depthMat, cv::Mat &edgeMap, int n, float jumpThresh)
{
    int height = depthMat.rows;
    int width = depthMat.cols;
    for(int y=n; y < height-n; ++y)
    {
        for(int x=n; x < width-n-20; ++x)
        {
            if(edgeMap.at<unsigned char>(y,x)==1)
                continue;
            bool bJump = true;
            //scanning horizontally
            for(int i=1; i < n; ++i)
            {
                for(int j=1; j < n; ++j)
                {
                    float diff = fabs(depthMat.at<float>(y,x-i) - depthMat.at<float>(y,x+j));
                    if(diff < jumpThresh)
                    {
                        bJump = false;
                        break;
                    }
                }
                if(bJump==false)
                    break;
            }
            //scanning vertically
            if(bJump==false)
            {
                bJump = true;
                for(int i=1; i < n; ++i)
                {
                    for(int j=1; j < n; ++j)
                    {
                        float diff = fabs(depthMat.at<float>(y-i,x) - depthMat.at<float>(y+j,x));
                        if(diff < jumpThresh)
                        {
                            bJump = false;
                            break;
                        }
                    }
                    if(bJump==false)
                        break;
                }
            }
            //left diagonically
            if(bJump==false)
            {
                bJump = true;
                for(int i=1; i < n; ++i)
                {
                    for(int j=1; j < n; ++j)
                    {
                        float diff = fabs(depthMat.at<float>(y-i,x-i) - depthMat.at<float>(y+j,x+j));
                        if(diff < jumpThresh)
                        {
                            bJump = false;
                            break;
                        }
                    }
                    if(bJump==false)
                        break;
                }
            }
            //right diagonically
            if(bJump==false)
            {
                for(int i=1; i < n; ++i)
                {
                    for(int j=1; j < n; ++j)
                    {
                        float diff = fabs(depthMat.at<float>(y-i,x+i) - depthMat.at<float>(y+j,x-j));
                        if(diff > jumpThresh)
                        {
                            bJump = true;
                            break;
                        }
                    }
                    if(bJump==false)
                        break;
                }
            }

            if(bJump==true)
            {
               edgeMap.at<unsigned char>(y,x)=1;
            }
        }
    }
}
/*
created at 08/16/2018
nMode = 0:scanning horizontally
nMode = 1: scanning vertically
nMode = 2: scanning left diagonally
nMode = 3: scanning right diagonally
*/
bool isEqualSlope(cv::Mat &depthMat, float slopeThresh, int x, int y,
                   int nPixels, bool bBefore, int nMode, float &mean)
{
    int sx = x;
    int sy = y;
    switch(nMode)
    {
    case 0://scanning horizontally
        if(bBefore==true)
        {
           sx = x-nPixels;
           for(int i=1; i < nPixels; ++i)
           {
              float s1 = depthMat.at<float>(sy,sx+i) - depthMat.at<float>(sy, sx+i+1);
              for(int j=i+1; j < nPixels-1; ++j)
              {
                  float s2 = depthMat.at<float>(sy, sx+j) - depthMat.at<float>(sy, sx+j+1);
                  if(fabs(s2-s1) > slopeThresh)
                      return false;
              }
              mean += s1;
           }
           return true;
        }
        else
        {
            for(int i=1; i < nPixels; ++i)
            {
               float s1 = depthMat.at<float>(sy,sx+i) - depthMat.at<float>(sy,sx+i+1);
               for(int j=i+1; j < nPixels-1; ++j)
               {
                   float s2 = depthMat.at<float>(sy,sx+j) - depthMat.at<float>(sy,sx+j+1);
                   if(fabs(s2-s1) > slopeThresh)
                       return false;
               }
               mean += s1;
            }
            return true;
        }
        break;
    case 1://scanning vertically
        if(bBefore==true)
        {
           sy = y-nPixels;
           for(int i=1; i < nPixels; ++i)
           {
              float s1 = depthMat.at<float>(sy-i,x) - depthMat.at<float>(sy-i+1,x);
              for(int j=i+1; j < nPixels; ++j)
              {
                  float s2 = depthMat.at<float>(sy-j,x) - depthMat.at<float>(sy-j+1,x);
                  if(fabs(s2-s1) > slopeThresh)
                      return false;
              }
              mean += s1;
           }
           return true;
        }
        else
        {
            sy = y;
            for(int i=1; i < nPixels; ++i)
            {
               float s1 = depthMat.at<float>(sy+i,x) - depthMat.at<float>(sy+i+1,x);
               for(int j=i+1; j < nPixels; ++j)
               {
                   float s2 = depthMat.at<float>(sy+j,x) - depthMat.at<float>(sy+j+1,x);
                   if(fabs(s2-s1) > slopeThresh)
                       return false;
               }
               mean += s1;
            }
            return true;
        }
        break;
    case 2://left diagonically
        if(bBefore==true)
        {
            sx = x-nPixels;
            sy = y-nPixels;
           for(int i=1; i < nPixels; ++i)
           {
              float s1 = depthMat.at<float>(sy+i,sx+i) - depthMat.at<float>(sy+i+1,x-nPixels+i+1);
              for(int j=i+1; j < nPixels-1; ++j)
              {
                  float s2 = depthMat.at<float>(y-nPixels+j,x-nPixels+j) - depthMat.at<float>(y-nPixels+j+1,x-nPixels+j+1);
                  if(fabs(s2-s1) > slopeThresh)
                      return false;
              }
              mean += s1;
           }
           return true;
        }
        else
        {
            for(int i=1; i < nPixels; ++i)
            {
               float s1 = depthMat.at<float>(y+i,x+i) - depthMat.at<float>(y+i+1,x+i+1);
               for(int j=i+1; j < nPixels-1; ++j)
               {
                   float s2 = depthMat.at<float>(y+j,x+j) - depthMat.at<float>(y+j+1,x+j+1);
                   if(fabs(s2-s1) > slopeThresh)
                       return false;
               }
               mean += s1;
            }
            return true;
        }
        break;
    case 3://right diagonically
        if(bBefore==true)
        {
           for(int i=1; i < nPixels; ++i)
           {
              float s1 = depthMat.at<float>(y-nPixels+i,x+nPixels-i) - depthMat.at<float>(y-nPixels+i+1,x+nPixels-i-1);
              for(int j=i+1; j < nPixels-1; ++j)
              {
                  float s2 = depthMat.at<float>(y-nPixels+j,x+nPixels-j) - depthMat.at<float>(y-nPixels+j+1,x+nPixels-j-1);
                  if(fabs(s2-s1) > slopeThresh)
                      return false;
              }
              mean += s1;
           }
           return true;
        }
        else
        {
            for(int i=1; i < nPixels; ++i)
            {
               float s1 = depthMat.at<float>(y+i,x-i) - depthMat.at<float>(y+i+1,x-i-1);
               for(int j=i+1; j < nPixels-1; ++j)
               {
                   float s2 = depthMat.at<float>(y+j,x-j) - depthMat.at<float>(y+j+1,x-j-1);
                   if(fabs(s2-s1) > slopeThresh)
                       return false;
               }
               mean += s1;
            }
            return true;
        }
        break;
    }
    return false;
}
/*
created at 08/18/2018
*/
bool corner_edge_horizon(cv::Mat &depthMat, int nSize, float slopeThresh1, float slopeThresh2, int cx, int cy)
{
    //check if this is equal slope
    //before side
    float mean1 = 0.0f;
    int count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy, cx-i)-depthMat.at<float>(cy, cx-i-1);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy, cx-j)-depthMat.at<float>(cy, cx-j-1);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean1 += s1;
        ++count;
    }
    mean1 /=count;
    //after size
    float mean2 = 0.0f;
    count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy, cx+i+1)-depthMat.at<float>(cy, cx+i);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy,cx+j+1)-depthMat.at<float>(cy,cx+j);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean2 += s1;
        ++count;
    }
    mean2 /=count;
    if(fabs(mean2-mean1)>slopeThresh2)
        return true;
    return false;
}
/*
created at 08/18/2018
*/
bool corner_edge_vertical(cv::Mat &depthMat, int nSize, float slopeThresh1, float slopeThresh2, int cx, int cy)
{
    //check if this is equal slope
    //before side
    float mean1 = 0.0f;
    int count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy-i, cx) - depthMat.at<float>(cy-i-1, cx);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy-j, cx)-depthMat.at<float>(cy-j-1, cx);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        ++count;
        mean1 += s1;
    }
    mean1 /=count;
    //after size
    count = 0;
    float mean2 = 0.0f;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy+i+1, cx)-depthMat.at<float>(cy+i, cx);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy+j+1, cx)-depthMat.at<float>(cy+j, cx);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean2 += s1;
        ++count;
    }
    mean2 /=count;
    if(fabs(mean2-mean1)>slopeThresh2)
        return true;
    return false;
}
/*
created at 08/18/2018
*/
bool corner_edge_left(cv::Mat &depthMat, int nSize, float slopeThresh1, float slopeThresh2, int cx, int cy)
{
    //check if this is equal slope
    //before side
    float mean1 = 0.0f;
    int count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy-i, cx-i) - depthMat.at<float>(cy-i-1, cx-i-i);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy-j, cx-j)-depthMat.at<float>(cy-j-1, cx-j-1);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean1 += s1;
        ++count;
    }
    mean1 /=count;
    //after size
    float mean2 = 0.0f;
    count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy+i+1, cx+i+1)-depthMat.at<float>(cy+i, cx+i);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy+j+1,cx+j+1)-depthMat.at<float>(cy+j,cx+j);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean2 += s1;
        ++count;
    }
    mean2 /=count;
    if(fabs(mean2-mean1)>slopeThresh2)
        return true;
    return false;
}
/*
created at 08/18/2018
*/
bool corner_edge_right(cv::Mat &depthMat, int nSize, float slopeThresh1, float slopeThresh2, int cx, int cy)
{
    //check if this is equal slope
    //before side
    float mean1 = 0.0f;
    int count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy-i, cx+i) - depthMat.at<float>(cy-i-1, cx+i+1);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy-j, cx+j)-depthMat.at<float>(cy-j-1, cx+j+1);
            if(fabs(s2-s1)>slopeThresh1)
                return false;
        }
        mean1 += s1;
        ++count;
    }
    mean1 /=count;
    //after size
    float mean2 = 0.0f;
    count = 0;
    for(int i=1; i < nSize; ++i)
    {
        float s1 = depthMat.at<float>(cy+i+1, cx-i-1)-depthMat.at<float>(cy+i, cx-i);
        for(int j=i+1; j < nSize; ++j)
        {
            float s2 = depthMat.at<float>(cy+j+1,cx-j-1)-depthMat.at<float>(cy+j,cx-j);
            if(fabs(s2-s1)>slopeThresh2)
                return false;
        }
        mean2 += s1;
        ++count;
    }
    mean2 /=count;
    if(fabs(mean2-mean1)>slopeThresh2)
        return true;
    return false;
}
/*
created at 08/17/2018
*/
void corner_edge(cv::Mat &depthMat, cv::Mat &edgeMap, int nPixels, float slopeThresh1, float slopeThresh2)
{
    int width = depthMat.cols;
    int height = depthMat.rows;
    for(int y=nPixels; y < height-nPixels; ++y)
    {
        for(int x=nPixels; x < width-nPixels; ++x)
        {
            if(edgeMap.at<unsigned char>(y,x)==1)
                continue;
            if(corner_edge_horizon(depthMat, nPixels, slopeThresh1, slopeThresh2, x, y))
            {
                edgeMap.at<unsigned char>(y,x)=1;
                continue;
            }
            if(corner_edge_vertical(depthMat, nPixels, slopeThresh1, slopeThresh2, x, y))
            {
                edgeMap.at<unsigned char>(y,x)=1;
                continue;
            }
            if(corner_edge_left(depthMat, nPixels, slopeThresh1, slopeThresh2, x, y))
            {
                edgeMap.at<unsigned char>(y,x)=1;
                continue;
            }
            if(corner_edge_right(depthMat, nPixels, slopeThresh1, slopeThresh2, x, y))
            {
                edgeMap.at<unsigned char>(y,x)=1;
                continue;
            }
        }
    }
}
/*
created at 08/24/2018
*/
float distanceFromPoint(float x1, float z1, float x2, float z2, float x)
{
    return z2 - (z2-z1)*(x2-x)/(x2-x);
}
/*
created at 08/22/2018
*/
bool isHorizonCurvedEdge(cv::Mat &depthMat, int x, int y, int nPixels, float straThresh)
{
    //check straight before side
    int x1 = x-nPixels;
    float z1 = depthMat.at<float>(y,x1);
    float z2 = depthMat.at<float>(y,x);
    int bBefore = 1;
    for(int i=1; i < nPixels; ++i)
    {
        float z = depthMat.at<float>(y,x-i);
        if(fabs(z-distanceFromPoint(x1, z1, x, z2, x-i)) > straThresh)
        {
            bBefore = 0;
            break;
        }
    }
    //check straight after side
    int bAfter = 1;
    x1 = x+nPixels;
    z1 = depthMat.at<float>(y,x1);
    for(int i=1; i < nPixels; ++i)
    {
        float z = depthMat.at<float>(y,x+i);
        if(fabs(z-distanceFromPoint(x1, z1, x, z2, x+i)) > straThresh)
        {
            bAfter = 0;
            break;
        }
    }
    if((bBefore+bAfter)==1)
        return true;
    return false;
}
/*
created at 08/23/2018
*/
bool isVerticalCurvedEdge(cv::Mat &depthMat, int x, int y, int nPixels, float straThresh)
{
    //check straight before side
    int y1 = y-nPixels;
    float z1 = depthMat.at<float>(y1,x);
    float z2 = depthMat.at<float>(y,x);
    int bBefore = 1;
    for(int i=1; i < nPixels; ++i)
    {
        float z = depthMat.at<float>(y-i,x);
        if(fabs(z-distanceFromPoint(y1, z1, y, z2, y-i)) > straThresh)
        {
            bBefore = 0;
            break;
        }
    }
    //check straight after side
    int bAfter = 1;
    y1 = y+nPixels;
    z1 = depthMat.at<float>(y1,x);
    for(int i=1; i < nPixels; ++i)
    {
        float z = depthMat.at<float>(y+i,x);
        if(fabs(z-distanceFromPoint(y1, z1, y, z2, y+i)) > straThresh)
        {
            bAfter = 0;
            break;
        }
    }
    if((bBefore+bAfter)==1)
        return true;
    return false;
}
/*
created at 08/23/2018
*/
bool isLeftCurvedEdge(cv::Mat &depthMat, int cx, int cy, int nPixels, float straThresh)
{
    //check straight before side
    int x1 = cx-nPixels;
    int y1 = cy-nPixels;
    float z1 = depthMat.at<float>(y1,x1);
    float cz = depthMat.at<float>(cy,cx);
    int bBefore = 1;
    for(int i=1; i < nPixels; ++i)
    {
        int x = cx - i;
        int y = cy - i;
        float z = depthMat.at<float>(y,x);
        float dist = distanceFromPoint(x1,z1,cx,cz,x);
        if(fabs(z-dist)>straThresh)
        {
            bBefore=0;
            break;
        }
    }
    int bAfter = 1;
    x1 = cx+nPixels;
    y1 = cy+nPixels;
    z1 = depthMat.at<float>(y1,x1);
    for(int i=1; i < nPixels; ++i)
    {
        int x = cx + i;
        int y = cy + i;
        float z = depthMat.at<float>(y,x);
        float dist = distanceFromPoint(x1,z1,cx,cz,x);
        if(fabs(z-dist)>straThresh)
        {
            bAfter=0;
            break;
        }
    }
    if((bBefore+bAfter)==1)
        return true;
    return false;
}
/*
created at 08/23/2018
*/
bool isRightCurvedEdge(cv::Mat &depthMat, int cx, int cy, int nPixels, float straThresh)
{
    //check straight before side
    int x1 = cx+nPixels;
    int y1 = cy-nPixels;
    float z1 = depthMat.at<float>(y1,x1);
    float cz = depthMat.at<float>(cy,cx);
    int bBefore = 1;
    for(int i=1; i < nPixels; ++i)
    {
        int x = cx + i;
        int y = cy - i;
        float z = depthMat.at<float>(y,x);
        float dist = distanceFromPoint(x1,z1,cx,cz,x);
        if(fabs(z-dist)>straThresh)
        {
            bBefore=0;
            break;
        }
    }
    int bAfter = 1;
    x1 = cx-nPixels;
    y1 = cy+nPixels;
    z1 = depthMat.at<float>(y1,x1);
    for(int i=1; i < nPixels; ++i)
    {
        int x = cx - i;
        int y = cy + i;
        float z = depthMat.at<float>(y,x);
        float dist = distanceFromPoint(x1,z1,cx,cz,x);
        if(fabs(z-dist)>straThresh)
        {
            bAfter=0;
            break;
        }
    }
    if((bBefore+bAfter)==1)
        return true;
    return false;
}
/*
created at 08/22/2018
*/
void curvedEdge(cv::Mat &depthMat, cv::Mat &edgeMat, int nPixels, float straThresh)
{
    int width = depthMat.cols;
    int height = depthMat.rows;
    for(int y=nPixels; y < height-nPixels; ++y)
    {
        for(int x=nPixels; x < width-nPixels; ++x)
        {
            if(edgeMat.at<unsigned char>(y,x)==1)
                continue;
            if(isHorizonCurvedEdge(depthMat, x, y, nPixels, straThresh))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            if(isVerticalCurvedEdge(depthMat, x, y, nPixels, straThresh))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            if(isLeftCurvedEdge(depthMat, x, y, nPixels, straThresh))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            if(isRightCurvedEdge(depthMat, x, y, nPixels, straThresh))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
        }
    }
}
/*
created at 08/24/2018
*/
bool isHorizonExtremum(cv::Mat &depthMat, int x, int y, int nPixels, float fExtremum)
{
    float fDepth = depthMat.at<float>(y,x);
    int minCount = 0;
    int maxCount = 0;
    int count = 0;
    for(int i=1; i < nPixels-1; ++i)
    {
        if(fExtremum*depthMat.at<float>(y,x-i) > fDepth && fExtremum*depthMat.at<float>(y,x+i) > fDepth)
        {
            ++minCount;
        }
        if(fExtremum*depthMat.at<float>(y,x-i) < fDepth && fExtremum*depthMat.at<float>(y,x+i) < fDepth)
        {
            ++maxCount;
        }
        ++count;
    }
    if(maxCount==count)
        return true;
    if(minCount==count)
        return true;
    return false;
}
/*
created at 08/24/2018
*/
bool isVerticalExtremum(cv::Mat &depthMat, int x, int y, int nPixels, float fExtremum)
{
    float fDepth = fExtremum*depthMat.at<float>(y,x);
    bool bMin = true;
    bool bMax = true;
    int minCount = 0;
    int maxCount = 0;
    int count = 0;
    for(int i=1; i < nPixels; ++i)
    {
        if(depthMat.at<float>(y-i,x) > fDepth && depthMat.at<float>(y+i,x) > fDepth)
            ++minCount;
        if(depthMat.at<float>(y-i,x) < fDepth && depthMat.at<float>(y+i,x) < fDepth)
            ++maxCount;
        ++count;
    }
    if(maxCount==count)
        return true;
    if(minCount==count)
        return true;
    return false;
}
/*
created at 08/24/2018
*/
bool isLeftDigExtremum(cv::Mat &depthMat, int x, int y, int nPixels, float fExtremum)
{
    float fDepth = fExtremum*depthMat.at<float>(y,x);
    int minCount = 0;
    int maxCount = 0;
    int count = 0;
    for(int i=1; i < nPixels; ++i)
    {
        if(depthMat.at<float>(y-i,x-i) > fDepth && depthMat.at<float>(y+i,x+i) > fDepth)
            ++minCount;
        if(depthMat.at<float>(y-i,x-i) < fDepth && depthMat.at<float>(y+i,x+i) < fDepth)
            ++maxCount;
        ++count;
    }
    if(maxCount==count)
        return true;
    if(minCount==count)
        return true;
    return false;
}
/*
created at 08/24/2018
*/
bool isRightDigExtremum(cv::Mat &depthMat, int x, int y, int nPixels, float fExtremum)
{
    float fDepth = fExtremum*depthMat.at<float>(y,x);
    int minCount = 0;
    int maxCount = 0;
    int count = 0;
    for(int i=1; i < nPixels; ++i)
    {
        if(depthMat.at<float>(y-i,x+i) > fDepth && depthMat.at<float>(y+i,x-i) > fDepth)
            ++minCount;
        if(depthMat.at<float>(y-i,x+i) < fDepth && depthMat.at<float>(y+i,x-i) < fDepth)
            ++maxCount;
        ++count;
    }
    if(maxCount==count)
        return true;
    if(minCount==count)
        return true;
    return false;
}
/*
created at 08/24/2018
*/
void extremumEdge(cv::Mat &depthMat, cv::Mat &edgeMat, int nPixels, float fExtremum)
{
    int width = depthMat.cols;
    int height = depthMat.rows;
    for(int y=nPixels; y<height-nPixels; ++y)
    {
        for(int x=nPixels; x<width-nPixels; ++x)
        {
            if(edgeMat.at<unsigned char>(y,x) == 1)
                continue;
            if(isHorizonExtremum(depthMat, x, y, nPixels, fExtremum)==true)
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            if(isVerticalExtremum(depthMat, x, y, nPixels, fExtremum))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            /*if(isLeftDigExtremum(depthMat, x, y, nPixels, fExtremum))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }
            if(isRightDigExtremum(depthMat, x, y, nPixels, fExtremum))
            {
                edgeMat.at<unsigned char>(y,x) = 1;
                continue;
            }*/
        }
    }
}
