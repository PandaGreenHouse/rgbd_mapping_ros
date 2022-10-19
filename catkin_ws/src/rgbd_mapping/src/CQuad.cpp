#include "CQuad.h"

CQuad::CQuad()
{

}

CQuad::CQuad(float z_bottom, float height)
{
    _z_bottom = z_bottom;
    _height = height;
    m_Vertices.resize(4);
    m_Line_list.resize(8);
    m_Line_list[0] = 0; m_Line_list[1] = 1;
    m_Line_list[2] = 1; m_Line_list[3] = 2;
    m_Line_list[4] = 2; m_Line_list[5] = 3;
    m_Line_list[6] = 3; m_Line_list[7] = 0;
}

void CQuad::setCoefficients(float c1, float c2, float c3, float c4)
{
    m_coefficients[0] = c1/c4;
    m_coefficients[1] = c2/c4;
    m_coefficients[2] = c3/c4;
    m_coefficients[3] = 1;
}

void CQuad::construct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::Vector3f &vNormal)
{
    Eigen::Vector3f mass_center(0.0f, 0.0f, 0.0f);
    for(int i=0; i < cloud_in->points.size(); ++i)
    {
       Eigen::Vector3f vertex = cloud_in->points[i].getVector3fMap();
       mass_center = (i*mass_center + vertex)/(i+1);
    }
    vNormal.normalize();
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f vDown(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f vForward = vUp.cross(vNormal);
    vForward.normalize();
    Eigen::Vector3f vBackward = -vForward;
    float forward_dist, back_dist, up_dist, down_dist;
    forward_dist = back_dist = up_dist = down_dist = FLT_MIN;
    for(int i=0; i < cloud_in->points.size(); ++i)
    {
       Eigen::Vector3f vec = cloud_in->points[i].getVector3fMap() - mass_center;
       float dot = vForward.dot(vec);
       if(dot > forward_dist)
          forward_dist = dot;
       dot = vBackward.dot(vec);
       if(dot > back_dist)
           back_dist = dot;
       dot = vUp.dot(vec);
       if(dot > up_dist)
           up_dist = dot;
       dot = vDown.dot(vec);
       if(dot > down_dist)
           down_dist = dot;
    }
    m_Vertices[0] = mass_center + forward_dist*vForward + down_dist*vDown;
    m_Vertices[1] = mass_center + back_dist*vBackward + down_dist*vDown;
    m_Vertices[2] = mass_center + back_dist*vBackward + up_dist*vUp;
    m_Vertices[3] = mass_center + forward_dist*vForward + up_dist*vUp;
    m_vNormal = vNormal;
    m_Vertices[0][2] = _z_bottom;
    m_Vertices[1][2] = _z_bottom;
    m_Vertices[2][2] = _z_bottom + _height;
    m_Vertices[3][2] = _z_bottom + _height;
}

float CQuad::distanceToQuad(std::vector<Eigen::Vector3f> &vertices)
{
    Eigen::Hyperplane<float,3> plane_ok = Eigen::Hyperplane<float,3>::Through(vertices[0], vertices[1], vertices[2]);
    Eigen::Vector3f vNormal(plane_ok.coeffs()[0], plane_ok.coeffs()[1], plane_ok.coeffs()[2]);
    float d = plane_ok.coeffs()[3]/vNormal.norm();
    vNormal.normalize();
    float dists[4];
    for(int i=0; i < 4; ++i)
    {
        dists[i] = vNormal.dot(m_Vertices[i]) + d;
        dists[i] = fabs(dists[i]);
    }
    dists[0] = MIN(dists[0], MIN(dists[1], dists[2]));
    dists[0] = MIN(dists[0], dists[3]);
    return dists[0];
}

void CQuad::getCenter(Eigen::Vector3f &vCenter)
{
    vCenter = (m_Vertices[0] + m_Vertices[1] + m_Vertices[2] + m_Vertices[3])/4;
}

bool CQuad::isOverlappedQuads(CQuad &quad, float dot_thresh_between_quads, float dist_thresh_between_quads)
{
    Eigen::Vector3f vNormal1, vNormal2;
    getNormal(vNormal1);
    quad.getNormal(vNormal2);
    float dot = fabs(vNormal1.dot(vNormal2));
    if(dot < dot_thresh_between_quads)
        return false;
    std::vector<Eigen::Vector3f> otherVertices;
    quad.getVertices(otherVertices);
    float dist_to = distanceToQuad(otherVertices);
    float dist_from = quad.distanceToQuad(m_Vertices);
    float dist = MIN(dist_to, dist_from);
    if(dist < dist_thresh_between_quads)
    {
        Eigen::Vector3f vCenter1, vCenter2;
        getCenter(vCenter1);
        quad.getCenter(vCenter2);
        Eigen::Vector3f vDisp = vCenter2 - vCenter1;
        Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f vForward = vUp.cross(vNormal1);
        vForward.normalize();
        float dist2 = fabs(vDisp.dot(vForward));
        Eigen::Vector3f vLeng[2];
        vLeng[0] = m_Vertices[1] - m_Vertices[0];
        vLeng[1] = otherVertices[1] - otherVertices[0];
        if(dist2 < ((vLeng[0].norm() + vLeng[1].norm())/2 + dist_thresh_between_quads))
        {
            return true;
        }
    }
    return false;
}

 bool CQuad::merge(CQuad &quad, float dot_thresh_between_quads,
                   float dist_thresh_between_quads, Eigen::Vector3f &vOffSet)
 {
     if(isOverlappedQuads(quad, dot_thresh_between_quads, dist_thresh_between_quads)==false)
         return false;
     /*Eigen::Vector3f vUpDir = m_Vertices[3] - m_Vertices[0];
     std::vector<Eigen::Vector3f> vertices;
     quad.getVertices(vertices);
     Eigen:Vector3f vDir = m_Vertices[0] - m_Vertices[1];
     vDir.normalize();
     float len = MAX(vDir.dot(vertices[0]-m_Vertices[0]),vDir.dot(vertices[1]-m_Vertices[0]));
     if(len > 0.0f)
     {
         m_Vertices[0] += len*vDir;
     }
     vDir = m_Vertices[1] - m_Vertices[0];
     vDir.normalize();
     len = MAX(vDir.dot(vertices[0]-m_Vertices[1]),vDir.dot(vertices[1]-m_Vertices[1]));
     if(len > 0.0f)
     {
         m_Vertices[1] += len*vDir;
     }
     m_Vertices[2] = m_Vertices[1] + vUpDir;
     m_Vertices[3] = m_Vertices[0] + vUpDir;
     m_Vertices[0][2] = _z_bottom;
     m_Vertices[1][2] = _z_bottom;
     m_Vertices[2][2] = _z_bottom + _height;
     m_Vertices[3][2] = _z_bottom + _height;*/
     *m_pCloud += *quad.getPointCloud();
     Eigen::Vector3f vNormals[2];
     getNormal(vNormals[0]);
     quad.getNormal(vNormals[1]);
     float dot = vNormals[0].dot(vNormals[1]);
     if(dot < 0.0f)
         vNormals[1] = -vNormals[1];
     Eigen::Vector3f vNormal = (vNormals[0] + vNormals[1])/2;
     vNormal.normalize();
     Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
     Eigen::Vector3f vForward = vUp.cross(vNormal);
     vForward.normalize();
     pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
     pCloud->resize(8);
     std::vector<Eigen::Vector3f> vertices;
     quad.getVertices(vertices);
     int j=0;
     for(int i=0; i < 4; ++i)
     {
         pCloud->points[j].x = m_Vertices[i][0];
         pCloud->points[j].y = m_Vertices[i][1];
         pCloud->points[j].z = m_Vertices[i][2];
         j++;
         pCloud->points[j].x = vertices[i][0];
         pCloud->points[j].y = vertices[i][1];
         pCloud->points[j].z = vertices[i][2];
         j++;
     }
     construct(pCloud, vNormal);
     /*Eigen::Vector3f vNormal;
     quad.getNormal(vNormal);
     Eigen::Vector3f p = vertices[0] + 0.5f*vNormal;
     Eigen::Vector3f interPos;
     if(getIntersection(vertices[0], p, interPos))
         vOffSet = vertices[0] - interPos;*/
     return true;
 }

void CQuad::getNormal(Eigen::Vector3f &vNormal)
{
    vNormal = m_vNormal;
}

float CQuad::getWidth()
{
    Eigen::Vector3f e;
    e = m_Vertices[1] - m_Vertices[0];
    return e.norm();
}

float CQuad::getHeight()
{
    Eigen::Vector3f e;
    e = m_Vertices[2] - m_Vertices[1];
    return e.norm();
}

void CQuad::getVertices(std::vector<Eigen::Vector3f> &vertices)
{
    vertices.resize(4);
    for(int i=0; i < 4; ++i)
    {
        vertices[i] = m_Vertices[i];
    }
}

void CQuad::getLineVertices(std::vector<Eigen::Vector3f> &vertices)
{
    vertices.resize(8);
    for(int i=0; i < 8; ++i)
    {
        vertices[i] = m_Vertices[m_Line_list[i]];
    }
}

void CQuad::update(float bottom, float height)
{
    m_Vertices[0][2] = bottom;
    m_Vertices[1][2] = bottom;
    m_Vertices[2][2] = bottom + height;
    m_Vertices[3][2] = bottom + height;
}

float CQuad::distanceFromPointToQuad(Eigen::Vector3f &pt, std::vector<Eigen::Vector3f> &vertices)
{
    Eigen::Hyperplane<float,3> plane_ok = Eigen::Hyperplane<float,3>::Through(vertices[0], vertices[1], vertices[2]);
    Eigen::Vector3f vNormal(plane_ok.coeffs()[0], plane_ok.coeffs()[1], plane_ok.coeffs()[2]);
    float d = plane_ok.coeffs()[3]/vNormal.norm();
    vNormal.normalize();
    float dist = m_vNormal.dot(pt) + d;
    return fabs(dist);
}

bool CQuad::correct(std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f vNormal, float dot_thresh)
{
    float dot = fabs(m_vNormal.dot(vNormal));
    if(dot > dot_thresh)
        return false;
    Eigen::Vector3f forward = m_Vertices[1] - m_Vertices[0];
    forward.normalize();
    Eigen::Vector3f backward = -forward;
    Eigen::Vector3f myVertices[4];
    for(int i=0; i < 4; ++i)
    {
        myVertices[i] = m_Vertices[i];
    }
    myVertices[0] += 0.1f*backward;
    myVertices[3] += 0.1f*backward;
    myVertices[1] += 0.1f*forward;
    myVertices[2] += 0.1f*forward;
    Eigen::Vector3f p1 = (myVertices[0] + myVertices[3])/2;
    Eigen::Vector3f p2 = (myVertices[1] + myVertices[2])/2;
    Eigen::Vector3f interPos;
    if(getIntersectionOfQuad(p1, p2, vertices, interPos)==false)
        return false;
    Eigen::Vector3f vec1 = interPos - p1;
    Eigen::Vector3f vec2 = interPos - p2;
    if(vec1.norm() < vec2.norm() && vec1.norm() < 0.3f)
    {
        myVertices[0] += vec1;
        myVertices[3] += vec1;
        m_Vertices[0] = myVertices[0];
        m_Vertices[3] = myVertices[3];
    }
    else if( vec2.norm() < 0.3f)
    {
        myVertices[1] += vec2;
        myVertices[2] += vec2;
        m_Vertices[1] = myVertices[1];
        m_Vertices[2] = myVertices[2];
    }
    return true;
}

void CQuad::correctingVertices(float camera_pos_z, float camera_height, float wall_height)
{
    float floor = camera_pos_z - camera_height;
    float delta = m_Vertices[0][2] - floor;
    for(int i=0; i < 2; ++i)
    {
        m_Vertices[i][2] = floor;
    }
    for(int i=2; i < 4; ++i)
    {
        m_Vertices[i][2] -= delta;
    }
}

void CQuad::set(std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &vNormal)
{
   Eigen::Vector3f vUp(0.0f,0.0f,1.0f);
   m_vNormal = vUp.cross(vNormal);
   m_vNormal = vUp.cross(m_vNormal);
   float dot = m_vNormal.dot(vNormal);
   if(dot < 0.0f)
       m_vNormal = -m_vNormal;
   m_vNormal.normalize();
   Eigen::Vector3f vHorizen = m_vNormal.cross(vUp);

   for(int i=0; i < 4; ++i)
   {
       m_Vertices[i] = vertices[i];
   }

}

bool CQuad::correct(CQuad &quad, float delta)
{
    std::vector<Eigen::Vector3f> vertices;
    quad.getVertices(vertices);
    Eigen::Vector3f vecs[2];
    vecs[0] = m_Vertices[0] - m_Vertices[1];
    vecs[1] = m_Vertices[1] - m_Vertices[0];
    vecs[0].normalize();
    vecs[1].normalize();
    for(int i=0; i < 2; ++i)
    {
        for(int j=0; j < 2; ++j)
        {
            Eigen::Vector3f vec = vertices[j] - m_Vertices[i];
            if(vec.norm() < delta)
            {
                m_Vertices[i] += vec.dot(vecs[i])*vecs[i];
                if(i==0)
                    m_Vertices[3]+=vec.dot(vecs[i])*vecs[i];
                if(i==1)
                    m_Vertices[2]+=vec.dot(vecs[i])*vecs[i];
                return true;
            }
        }
    }
    return false;
}

float CQuad::getDistanceFromPoint(Eigen::Vector3f &pt)
{
    Eigen::Vector3f vNormal(m_coefficients[0], m_coefficients[1], m_coefficients[2]);
    float dist = (pt.dot(vNormal) + 1)/vNormal.norm();
    return fabs(dist);
}

bool CQuad::getIntersection(Eigen::Vector3f &p1,Eigen::Vector3f &p2, Eigen::Vector3f &p3)
{
    ParametrizedLine<float,3> pline = ParametrizedLine<float,3>::Through(p1,p2);
    Eigen::Hyperplane<float,3> focalPlane = Eigen::Hyperplane<float,3>::Through(m_Vertices[0], m_Vertices[1], m_Vertices[2]);
    //double intersection = pline.intersection(focalPlane);
    Eigen::Vector3f interPos = pline.intersectionPoint(focalPlane);//fabs(intersection)*((p2-p1).normalized()) + p1;
    //if(fabs(intersection) > (p2 - p1).norm())
    //    return false;
    float len = MAX((interPos - p1).norm(), (interPos - p2).norm());
    if(len > (p2-p1).norm())
        return false;
    std::vector<Eigen::Vector3f> vts;
    vts.resize(5);
    for(int i=0; i < 4; ++i)
    {
        vts[i] = m_Vertices[i];
    }
    vts[4] = m_Vertices[0];
    for(int i=0; i < 4; ++i)
    {
        if((interPos-vts[i]).dot((vts[i+1]-vts[i])) < 0.0f)
            return false;
    }
    p3 = interPos;
    return true;
}

bool CQuad::checkAlignment(CQuad &quad, float lower, float upper)
{
    Eigen::Vector3f vNormal1, vNormal2;
    getNormal(vNormal1);
    quad.getNormal(vNormal2);
    if(fabs(vNormal1.dot(vNormal2)) < 0.1f)
        return true;
    std::vector<Eigen::Vector3f> otherVertices;
    quad.getVertices(otherVertices);
    float dist_to = distanceToQuad(otherVertices);
    float dist_from = quad.distanceToQuad(m_Vertices);
    float dist = MIN(dist_to, dist_from);
    if(dist > lower && dist < upper)
    {
        Eigen::Vector3f vCenter1, vCenter2;
        getCenter(vCenter1);
        quad.getCenter(vCenter2);
        Eigen::Vector3f vDisp = vCenter2 - vCenter1;
        Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f vForward = vUp.cross(vNormal1);
        vForward.normalize();
        float dist2 = fabs(vDisp.dot(vForward));
        Eigen::Vector3f vLeng[2];
        vLeng[0] = m_Vertices[1] - m_Vertices[0];
        vLeng[1] = otherVertices[1] - otherVertices[0];
        if(dist2 < (vLeng[0].norm() + vLeng[1].norm())/2)
        {
            return false;
        }
    }
    return true;
}

float CQuad::getDensity()
{
    float w = (m_Vertices[1] - m_Vertices[0]).norm();
    float h = (m_Vertices[3] - m_Vertices[0]).norm();
    return m_pCloud->points.size()/(w*h);
}

void CQuad::constructGround(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
    Eigen::Vector3f vDir0(-1.0f, 0.0f, 0.0f);
    Eigen::Vector3f vDir1(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f vDir2(0.0f, -1.0f, 0.0f);
    Eigen::Vector3f vDir3(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f vDir4(0.0f, 0.0f, -1.0f);
    float fDists[5];
    fDists[0] = fDists[1] = fDists[2] = fDists[3] = fDists[4] = 0.0f;
    Eigen::Vector3f vCenter(0.0f,0.0f,0.0f);
    for(int i=0; i < pCloud->points.size(); ++i)
    {
        Eigen::Vector3f v = pCloud->points[i].getVector3fMap();
        vCenter = (i*vCenter + v)/(i+1);
    }
    for(int i=0; i < pCloud->points.size(); ++i)
    {
        Eigen::Vector3f vDist = pCloud->points[i].getVector3fMap() - vCenter;
        float dot = vDist.dot(vDir0);
        if(dot > fDists[0])
        {
            fDists[0] = dot;
        }
        dot = vDist.dot(vDir1);
        if(dot > fDists[1])
        {
            fDists[1] = dot;
        }
        dot = vDist.dot(vDir2);
        if(dot > fDists[2])
        {
            fDists[2] = dot;
        }
        dot = vDist.dot(vDir3);
        if(dot > fDists[3])
        {
            fDists[3] = dot;
        }
        dot = vDist.dot(vDir4);
        if(dot > fDists[4])
        {
            fDists[4] = dot;
        }
    }
    m_Vertices[0] = vCenter + fDists[0]*vDir0 + fDists[2]*vDir2;// + fDists[4]*vDir4;
    m_Vertices[1] = vCenter + fDists[1]*vDir1 + fDists[2]*vDir2;// + fDists[4]*vDir4;
    m_Vertices[2] = vCenter + fDists[1]*vDir1 + fDists[3]*vDir3;// + fDists[4]*vDir4;
    m_Vertices[3] = vCenter + fDists[0]*vDir0 + fDists[3]*vDir3;// + fDists[4]*vDir4;
}

float CQuad::getBottom()
{
    return m_Vertices[0][2];
}
