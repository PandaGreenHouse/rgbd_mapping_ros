#include "CBoundingBox.h"

CBoundingBox::CBoundingBox()
{
    //front and back quads
    for(int i=0; i < 4; i++)
    {
       quad_list[i] = i;
       quad_list[i+4] = i+4;
    }
    //left quad 0, 4, 7, 3
    quad_list[8] = 0;
    quad_list[9] = 4;
    quad_list[10] = 7;
    quad_list[11] = 3;
    //right quad 1, 5, 6, 2
    quad_list[12] = 1;
    quad_list[13] = 5;
    quad_list[14] = 6;
    quad_list[15] = 2;
    //top quad 3, 2, 6, 7
    quad_list[16] = 3;
    quad_list[17] = 2;
    quad_list[18] = 6;
    quad_list[19] = 7;
    //bottom quad 0,1,5,4
    quad_list[20] = 0;
    quad_list[21] = 1;
    quad_list[22] = 5;
    quad_list[23] = 4;
    //front quad
    line_list[0] = 0; line_list[1] = 1;
    line_list[2] = 1; line_list[3] = 2;
    line_list[4] = 2; line_list[5] = 3;
    line_list[6] = 3; line_list[7] = 0;
    //back quad
    line_list[8] = 4; line_list[9] = 5;
    line_list[10] = 5; line_list[11] = 6;
    line_list[12] = 6; line_list[13] = 7;
    line_list[14] = 7; line_list[15] = 4;
    //left quad
    line_list[16] = 0; line_list[17] = 3;
    line_list[18] = 3; line_list[19] = 7;
    line_list[20] = 7; line_list[21] = 4;
    line_list[22] = 4; line_list[23] = 0;
    //right quad
    line_list[24] = 1; line_list[25] = 5;
    line_list[26] = 5; line_list[27] = 6;
    line_list[28] = 6; line_list[29] = 2;
    line_list[30] = 2; line_list[31] = 1;
    //top quad
    line_list[32] = 3; line_list[33] = 2;
    line_list[34] = 2; line_list[35] = 6;
    line_list[36] = 6; line_list[37] = 7;
    line_list[38] = 7; line_list[39] = 3;
    //bottom quad
    line_list[40] = 0; line_list[41] = 1;
    line_list[42] = 1; line_list[43] = 5;
    line_list[44] = 5; line_list[45] = 4;
    line_list[46] = 4; line_list[47] = 0;
    for(int i=0; i < 8; ++i)
    {
        vertices[i][0] = 0.0f;
        vertices[i][1] = 0.0f;
        vertices[i][2] = 0.0f;
    }
    m_edges[0] = 1; m_edges[1] = 3; m_edges[2] = 4;
    m_edges[3] = 0; m_edges[4] = 2; m_edges[5] = 5;
    m_edges[6] = 1; m_edges[7] = 3; m_edges[8] = 6;
    m_edges[9] = 0; m_edges[10] = 2; m_edges[11] = 7;

    m_edges[12] = 5; m_edges[13] = 7; m_edges[14] = 0;
    m_edges[15] = 4; m_edges[16] = 6; m_edges[17] = 1;
    m_edges[18] = 5; m_edges[19] = 7; m_edges[20] = 2;
    m_edges[21] = 4; m_edges[22] = 6; m_edges[23] = 3;
    _count = 0;
}

void CBoundingBox::constructOBB(std::vector<Eigen::Vector3f> obb_vertices)
{
    for(int i=0;i<8; ++i)
    {
        vertices[i] = obb_vertices[i];
    }
    computingQuadNormals();
}

void CBoundingBox::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
    m_pCloud = pCloud;
}

void CBoundingBox::construct(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, Eigen::Vector3f &vNormal_Wall, int nFrameId)
{
    Eigen::Vector3f mass_center(0.0f, 0.0f, 0.0f);
    for(int i=0; i < pCloud->points.size(); ++i)
    {
       Eigen::Vector3f vertex = pCloud->points[i].getVector3fMap();
       mass_center = (i*mass_center + vertex)/(i+1);
    }
    vNormal_Wall.normalize();
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f vDown(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f vForward = vNormal_Wall;
    Eigen::Vector3f vBackward = -vForward;
    Eigen::Vector3f vLeft = vForward.cross(vUp);
    Eigen::Vector3f vRight = -vLeft;
    vLeft.normalize();
    vRight.normalize();
    Eigen::Vector3f vertex, vec;
    float dist_up, dist_down, dist_forward, dist_backward, dist_left, dist_right;
    dist_up = dist_down = dist_forward = dist_backward = dist_left = dist_right = FLT_MIN;
    for(int i=0; i < pCloud->points.size(); ++i)
    {
      vertex = pCloud->points[i].getVector3fMap();
      vec = vertex - mass_center;
      dist_forward = MAX(vec.dot(vForward), dist_forward);
      dist_backward = MAX(vec.dot(vBackward), dist_backward);
      dist_up = MAX(vec.dot(vUp), dist_up);
      dist_down = MAX(vec.dot(vDown), dist_down);
      dist_left = MAX(vec.dot(vLeft), dist_left);
      dist_right = MAX(vec.dot(vRight), dist_right);
    }
    /*dist_left *= 0.8f;
    dist_right *= 0.8f;
    dist_forward *= 0.8f;
    dist_backward *= 0.8f;
    dist_up *= 0.8f;
    dist_down *= 0.8f;*/
    vertices[0] = mass_center + dist_left * vLeft + dist_down * vDown + dist_forward * vForward;
    vertices[1] = mass_center + dist_right * vRight + dist_down * vDown + dist_forward * vForward;
    vertices[2] = mass_center + dist_right * vRight + dist_up * vUp + dist_forward * vForward;
    vertices[3] = mass_center + dist_left * vLeft + dist_up * vUp + dist_forward * vForward;

    vertices[4] = mass_center + dist_left * vLeft + dist_down * vDown + dist_backward * vBackward;
    vertices[5] = mass_center + dist_right * vRight + dist_down * vDown + dist_backward * vBackward;
    vertices[6] = mass_center + dist_right * vRight + dist_up * vUp + dist_backward * vBackward;
    vertices[7] = mass_center + dist_left * vLeft + dist_up * vUp + dist_backward * vBackward;
    computingQuadNormals();
    Eigen::Vector3f v_min(FLT_MAX, FLT_MAX, FLT_MAX);
    Eigen::Vector3f v_max(FLT_MIN, FLT_MIN, FLT_MIN);
    for(int i=0; i < 8; ++i)
    {
        v_min[0] = MIN(v_min[0], vertices[i][0]);
        v_min[1] = MIN(v_min[1], vertices[i][1]);
        v_min[2] = MIN(v_min[2], vertices[i][2]);
        v_max[0] = MAX(v_max[0], vertices[i][0]);
        v_max[1] = MAX(v_max[1], vertices[i][1]);
        v_max[2] = MAX(v_max[2], vertices[i][2]);
    }
    min_point = v_min;
    max_point = v_max;
    _frame_id = nFrameId;
    _count++;
}

bool CBoundingBox::isInBox(Eigen::Vector3f &vertex)
{
    int count = 0;
    for(int i=0; i < 6; ++i)
    {
        int k = quad_list[4*i];
        Eigen::Vector3f vec = vertex - vertices[k];
        Eigen::Vector3f vNormal = vNormals[i];
        float dot = vec.dot(vNormal);
        if(dot > 0.0f)
            return false;
    }
    return true;
}

bool CBoundingBox::isOverlapped(CBoundingBox &box)
{
    /*std::vector<Eigen::Vector3f> box_vertices;
    box.getVertices(box_vertices);
    //checking box overlapped
    for(int i=0; i < 8; ++i)
    {
        if(isInBox(box_vertices[i])==true)
            return true;
    }
    for(int i=0; i < 8; ++i)
    {
        if(box.isInBox(vertices[i])==true)
            return true;
    }
    return false;*/
    //check sphere overlap
    /*float radius1 = (vertices[0] - vertices[6]).norm()/2;
    float radius2 = (box_vertices[0] - box_vertices[6]).norm()/2;
    Eigen::Vector3f vCenter1, vCenter2;
    getCenter(vCenter1);
    box.getCenter(vCenter2);
    if((vCenter2-vCenter1).norm() > radius1+radius2)
        return false;
    return true;*/
    //checking AABB overlap
    /*float x_min = MIN(min_point[0], box.min_point[0]);
    float y_min = MIN(min_point[1], box.min_point[1]);
    float z_min = MIN(min_point[2], box.min_point[2]);
    float x_max = MAX(max_point[0], box.max_point[0]);
    float y_max = MAX(max_point[1], box.max_point[1]);
    float z_max = MAX(max_point[2], box.max_point[2]);
    if((x_max-x_min) > (max_point[0] - min_point[0] + box.max_point[0] - box.min_point[0]))
        return false;
    if((y_max-y_min) > (max_point[1] - min_point[1] + box.max_point[1] - box.min_point[1]))
        return false;
    if((z_max-z_min) > (max_point[2] - min_point[2] + box.max_point[2] - box.min_point[2]))
        return false;
    return true;*/
    //20180515
    /*std::vector<Eigen::Vector3f> other_vertices;
    box.getVertices(other_vertices);
    float len1 = (vertices[1]-vertices[0]).norm() + (other_vertices[1]-other_vertices[0]).norm();
    float len2 = MAX((other_vertices[1]-vertices[0]).norm(), (vertices[1]-other_vertices[0]).norm());
    if(len2 > len1)
        return false;
    len1 = (vertices[3]-vertices[0]).norm() + (other_vertices[3]-other_vertices[0]).norm();
    len2 = MAX((other_vertices[3]-vertices[0]).norm(), (vertices[3]-other_vertices[0]).norm());
    if(len2 > len1)
        return false;
    len1 = (vertices[4]-vertices[0]).norm() + (other_vertices[4]-other_vertices[0]).norm();
    len2 = MAX((other_vertices[4]-vertices[0]).norm(), (vertices[4]-other_vertices[0]).norm());
    if(len2 > len1)
        return false;
    return true;*/
    //2018/05/21
    std::vector<Eigen::Vector3f> other_vertices;
    box.getVertices(other_vertices);
    std::vector<Eigen::Vector3f> vertices_merged;
    for(int i=0; i < 8; ++i)
    {
        vertices_merged.push_back(vertices[i]);
        vertices_merged.push_back(other_vertices[i]);
    }
    std::vector<Eigen::Vector3f> newVertices;
    Eigen::Vector3f vDir = vertices[0] - vertices[4];
    vDir.normalize();
    computingBoundingBox(vertices_merged, vDir, newVertices);
    float len1 = (vertices[1] - vertices[0]).norm() + (other_vertices[1] - other_vertices[0]).norm();
    float len2 = (newVertices[1] - newVertices[0]).norm();
    if(len2 > len1)
        return false;
    len1 = (vertices[3] - vertices[0]).norm() + (other_vertices[3] - other_vertices[0]).norm();
    len2 = (newVertices[3] - newVertices[0]).norm();
    if(len2 > len1)
        return false;
    len1 = (vertices[4] - vertices[0]).norm() + (other_vertices[4] - other_vertices[0]).norm();
    len2 = (newVertices[4] - newVertices[0]).norm();
    if(len2 > len1)
        return false;
    return true;
}

void CBoundingBox::getCenter(Eigen::Vector3f &vCenter)
{
    vCenter[0] = 0.0f;
    vCenter[1] = 0.0f;
    vCenter[2] = 0.0f;
    for(int i=0; i < 8; ++i)
    {
        vCenter[0] += vertices[i][0];
        vCenter[1] += vertices[i][1];
        vCenter[2] += vertices[i][2];
    }
    vCenter[0] = vCenter[0]/8;
    vCenter[1] = vCenter[1]/8;
    vCenter[2] = vCenter[2]/8;
}

float CBoundingBox::getIntersection(Eigen::Vector3f &vRayPos, Eigen::Vector3f &vRayDir, Eigen::Vector3f &interPos)
{
    float dist = 0.0f;
    Eigen::Vector3f targPos = vRayPos + 1000*vRayDir;
    vRayDir.normalize();
    for(int i=0; i < 6; ++i)
    {
        float dot = vRayDir.dot(vNormals[i]);
        if(dot > 0.6f)
        {
            std::vector<Eigen::Vector3f> my_vertices;
            my_vertices.resize(4);
            for(int j=0; j < 4; j++)
            {
                my_vertices[j] = vertices[quad_list[4*i+j]];
            }
            getIntersectionOfQuad(vRayPos, targPos, my_vertices, interPos);
            dist = (interPos - vRayPos).norm();
            return dist;
        }
    }
    return dist;
}

bool CBoundingBox::merge(CBoundingBox &box, Eigen::Vector3f &vNormal_Wall, int toleranceFrames)
{
    if(isOverlapped(box)==false)
        return false;
    if(fabs((float)_frame_id-(float)box.getFrameId()) > toleranceFrames)
        return true;
    *m_pCloud += *(box.getCloudPtr());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i < 8; ++i)
    {
        pcl::PointXYZ p1(vertices[i][0], vertices[i][1], vertices[i][2]);
        pCloud->points.push_back(p1);
        pcl::PointXYZ p2(box.vertices[i][0], box.vertices[i][1], box.vertices[i][2]);
        pCloud->points.push_back(p2);
    }
    construct(pCloud, vNormal_Wall, _frame_id);
    return true;
    /*Eigen::Vector3f vCenters[3];
    getCenter(vCenters[0]);
    box.getCenter(vCenters[1]);
    vCenters[2] = (vCenters[0] + vCenters[1])/2;
    Eigen::Vector3f my_normals[6];
    my_normals[0] = vNormal_Wall;
    my_normals[0].normalize();
    my_normals[1] = -my_normals[0];
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    my_normals[2] = my_normals[0].cross(vUp);
    my_normals[2].normalize();
    my_normals[3] = -my_normals[2];
    my_normals[4] = vUp;
    my_normals[5] = -vUp;
    Eigen::Vector3f vDisps[6];
    for(int i=0; i < 6; ++i)
    {
        Eigen::Vector3f interPos1, interPos2;
        float dist1 = getIntersection(vCenters[0], vNormals[i], interPos1);
        vDisps[i] = interPos1 - vCenters[0];
        //float dist2 = box.getIntersection(vCenters[2], my_normals[i], interPos2);
        /*if(dist2 > dist1)
        {
            vDisps[i] = interPos2 - vCenters[2];
        }
        else
            vDisps[i] = interPos1 - vCenters[2];*/
    /*}
    vertices[0] = vCenters[0] + vDisps[2] + vDisps[5] + vDisps[0];
    vertices[1] = vCenters[0] + vDisps[3] + vDisps[5] + vDisps[0];
    vertices[2] = vCenters[0] + vDisps[3] + vDisps[4] + vDisps[0];
    vertices[3] = vCenters[0] + vDisps[2] + vDisps[4] + vDisps[0];

    vertices[4] = vCenters[0] + vDisps[2] + vDisps[5] + vDisps[1];
    vertices[5] = vCenters[0] + vDisps[3] + vDisps[5] + vDisps[1];
    vertices[6] = vCenters[0] + vDisps[3] + vDisps[4] + vDisps[1];
    vertices[7] = vCenters[0] + vDisps[2] + vDisps[4] + vDisps[1];
    computingQuadNormals();
    return true;*/
}

void CBoundingBox::getVertices(std::vector<Eigen::Vector3f> &my_vertices)
{
    my_vertices.resize(8);
    for(int i=0;i < 8; ++i)
    {
        my_vertices[i] = vertices[i];
    }
}

void CBoundingBox::getWireFrame(std::vector<Eigen::Vector3f> &line_vertices)
{
    for(int i=0; i < 48; ++i)
    {
        Eigen::Vector3f v = vertices[line_list[i]];
        line_vertices.push_back(v);
    }
}

void CBoundingBox::computingQuadNormals()
{
    Eigen::Vector3f vCenter(0.0f, 0.0f, 0.0f);
    for(int i=0; i < 8; ++i)
    {
        vCenter += vertices[i];
    }
    vCenter /= 8;
    for(int i=0; i < 6; ++i)
    {
        Eigen::Vector3f v1 = vertices[quad_list[4*i]];
        Eigen::Vector3f v2 = vertices[quad_list[4*i+1]];
        Eigen::Vector3f v3 = vertices[quad_list[4*i+2]];
        Eigen::Vector3f v4 = vertices[quad_list[4*i+3]];
        Eigen::Vector3f e1 = v2 - v1;
        Eigen::Vector3f e2 = v4 - v1;
        Eigen::Vector3f vNormal = e1.cross(e2);
        Eigen::Vector3f disp = v1 - vCenter;
        float dot = disp.dot(vNormal);
        if(dot < 0.0f)
            vNormal = -vNormal;
        vNormal.normalize();
        vNormals[i] = vNormal;
    }
}

void CBoundingBox::correctingVertices(float camera_pos_z, float camera_height, float wall_height)
{
    float floor = camera_pos_z - camera_height;
    float delta = vertices[0][2] - floor;
    for(int i=0; i < 2; ++i)
    {
        vertices[i][2] = floor;
        vertices[i+4][2] = floor;
    }
    for(int i=2; i < 4; ++i)
    {
        vertices[i][2] -= delta;
        vertices[i+4][2] -= delta;
    }
}

float CBoundingBox::getDepth()
{
    float norms[3];
    norms[0] = (vertices[1] - vertices[0]).norm();
    norms[1] = (vertices[3] - vertices[0]).norm();
    norms[2] = (vertices[4] - vertices[0]).norm();
    float depth = norms[2];//MIN(norms[0], MIN(norms[1], norms[2]));
    return depth;
}

float CBoundingBox::getTop(float zBottom)
{
    return (vertices[3][2] - zBottom);
}

float CBoundingBox::getBottom()
{
    return vertices[0][2];
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CBoundingBox::getCloudPtr()
{
    return m_pCloud;
}

int CBoundingBox::isStand()
{
    float height = fabs(vertices[3][2] - vertices[0][2]);
    float width = (vertices[1] - vertices[0]).norm();
    float depth = (vertices[4] - vertices[0]).norm();
    if(height > width || height > depth)
        return 1;
    return 0;
}

float CBoundingBox::getMaxDimension()
{
    float norms[3];
    norms[0] = (vertices[1] - vertices[0]).norm();
    norms[1] = (vertices[3] - vertices[0]).norm();
    norms[2] = (vertices[4] - vertices[0]).norm();
    return MAX(norms[0], MAX(norms[1], norms[2]));
}

float CBoundingBox::getMinDimension()
{
    float norms[3];
    norms[0] = (vertices[1] - vertices[0]).norm();
    norms[1] = (vertices[3] - vertices[0]).norm();
    norms[2] = (vertices[4] - vertices[0]).norm();
    return MIN(norms[0], MIN(norms[1], norms[2]));
}

void CBoundingBox::getBackQuad(Eigen::Vector3f &cam_position, std::vector<Eigen::Vector3f> &out_vertices, Eigen::Vector3f &vOutNormal)
{
    Eigen::Vector3f vec1 = cam_position - vertices[4];
    Eigen::Vector3f vec2 = vertices[0] - vertices[4];
    float dot = vec1.dot(vec2);
    if(dot > 0.0f)
    {
        for(int i=0; i < 4; ++i)
        {
            out_vertices[i] = vertices[i+4];
        }
        vOutNormal = vertices[0] - vertices[4];
        vOutNormal.normalize();
    }
    else
    {
        for(int i=0; i < 4; ++i)
        {
            out_vertices[i] = vertices[i];
        }
        vOutNormal = vertices[4] - vertices[0];
        vOutNormal.normalize();
    }
}

bool CBoundingBox::checkIntersectWall(CQuad &quad)
{
    std::vector<Eigen::Vector3f> myVertices;
    myVertices.resize(5);
    myVertices[0] = vertices[3];
    myVertices[1] = vertices[2];
    myVertices[2] = vertices[6];
    myVertices[3] = vertices[7];
    myVertices[4] = vertices[3];
    for(int i=0; i < 4; ++i)
    {
        Eigen::Vector3f interPos;
        if(quad.getIntersection(myVertices[i], myVertices[i+1], interPos))
            return true;
    }
    return false;
}

float CBoundingBox::getDensity()
{
    float width = (vertices[1] - vertices[0]).norm();
    float height = (vertices[3] - vertices[0]).norm();
    float depth = (vertices[4] - vertices[0]).norm();
    return m_pCloud->points.size()/(width*height*depth);
}

float CBoundingBox::getHeight()
{
   return fabs(vertices[3][2]-vertices[0][2]);
}
