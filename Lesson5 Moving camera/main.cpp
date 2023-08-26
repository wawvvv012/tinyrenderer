#include <tgaimage.h>
#include <ctime>
#include <iostream>
#include <geometry.h>
#include <model.h>
#include <algorithm>

Model* model = NULL;
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int width = 800;
const int height = 800;
const int depth = 255;

Vec3f light_dir = Vec3f(1,-1,1).normalize(); //��Դ���������ķ���Ҳ���ǹ�Դ�����򣨱ʼǣ�
Vec3f eye(2, 1, 3);
Vec3f center(0, 0, 1);

//����Ⱦ��
void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    //б�ʴ���1��С��-1������x y
    if (std::abs(x1 - x0) < std::abs(y1 - y0)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror2 = std::abs(dy) * 2;
    int error2 = 0;
    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
        error2 += derror2;
        if (error2 > dx) {
            y += (y1 > y0) ? 1 : -1;
            error2 -= 2 * dx;
        }
    }
}

//vector->matrix 3*1��������Ϊ4*1������������ʽ
Matrix v2m(Vec3f v) {
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}

//NDC����-����Ļ���꣬��(x,y)Ϊԭ�㡢w hΪ��ߵ���Ļ����
Matrix viewport(int x, int y, int w, int h) {
    Matrix m = Matrix::identity(4);
    m[0][3] = x + w / 2.f;
    m[1][3] = y + h / 2.f;
    m[2][3] = depth / 2.f;

    m[0][0] = w / 2.f;
    m[1][1] = h / 2.f;
    m[2][2] = depth / 2.f;
    return m;
}

//����ӽ�
Matrix lookat(Vec3f eye, Vec3f center, Vec3f up) {
    //��������ϵ
    Vec3f z = (eye - center).normalize(); //v��
    Vec3f x = (up ^ z).normalize(); //r��
    Vec3f y = (z ^ x).normalize(); //u��
    Matrix res = Matrix::identity(4);
    for (int i = 0; i < 3; i++) {
        res[0][i] = x[i];
        res[1][i] = y[i];
        res[2][i] = z[i];
        res[i][3] = -center[i];
    }
    return res;
}
//���������� ptsָ����Ļ����screenCoords[3]
Vec3f barycentric(Vec3f* pts, Vec3f P) {
    float xa = pts[0].x;
    float ya = pts[0].y;
    float xb = pts[1].x;
    float yb = pts[1].y;
    float xc = pts[2].x;
    float yc = pts[2].y;
    float x = P.x;
    float y = P.y;

    float gamma = static_cast<float>((ya - yb) * x + (xb - xa) * y + xa * yb - xb * ya) / static_cast<float>((ya - yb) * xc + (xb - xa) * yc + xa * yb - xb * ya);
    float beta = static_cast<float>((ya - yc) * x + (xc - xa) * y + xa * yc - xc * ya) / static_cast<float>((ya - yc) * xb + (xc - xa) * yb + xa * yc - xc * ya);
    float alpha = 1 - gamma - beta;

    return Vec3f(alpha, beta, gamma);
}

//��������Ⱦ�� zbuffer��
//pts = &screenCoords[0]��pts[1] = *(pts + 1)��ptsָ����Ļ����screenCoords[3]
void triangle(Vec3f* pts, float* zbuffer, TGAImage& image, const TGAColor& color) {
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);
    Vec2f bboxMax(0, 0);

    bboxMin.x = floor(std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMin.y = floor(std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y }));
    bboxMax.x = ceil(std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMax.y = ceil(std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y }));

    for (int i = bboxMin.x; i <= bboxMax.x; i++) {
        for (int j = bboxMin.y; j <= bboxMax.y; j++) {
            Vec3f P(i + 0.5, j + 0.5, 0.0); //�ж����������ǲ�������������
            Vec3f baryCoord = barycentric(pts, P);
            //����ϵ�����Ǹ�������������
            if (baryCoord.x < -0.01 || baryCoord.y < -0.01 || baryCoord.z < -0.01) {
                continue;
            }
            //��ʱ����������
            //z-buffer�жϸ��������ڵ������Ƿ�ɼ�
            //���������ֵ�õ����ص�P�����ֵz ��Χ[-1,1]
            P.z = baryCoord.x * pts[0].z + baryCoord.y * pts[1].z + baryCoord.z * pts[2].z;
            //��Ϊ����-z����zԽ��--�������Խ��
            //x = idx % width��y = idx / width��idx = x + y * width
            if (zbuffer[int(P.x + P.y * width)] < P.z) {
                zbuffer[int(P.x + P.y * width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

//����ӳ�� ��������Ⱦ��,uv��ָ��vec2f [3]�����ָ��
void triangle(Vec3f* pts, float* zbuffer, TGAImage& image, Vec2i* uv,float intensity) {
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);
    Vec2f bboxMax(0, 0);

    bboxMin.x = floor(std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMin.y = floor(std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y }));
    bboxMax.x = ceil(std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMax.y = ceil(std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y }));

    for (int i = bboxMin.x; i <= bboxMax.x; i++) {
        for (int j = bboxMin.y; j <= bboxMax.y; j++) {
            Vec3f P(i + 0.5, j + 0.5, 0.0); //�ж����������ǲ�������������
            Vec3f baryCoord = barycentric(pts, P);
            //����ϵ�����Ǹ�������������
            if (baryCoord.x < -0.01 || baryCoord.y < -0.01 || baryCoord.z < -0.01) {
                continue;
            }
            //��ʱ����������
            //z-buffer�жϸ��������ڵ������Ƿ�ɼ�
            //���������ֵ�õ����ص�P�����ֵz ��Χ[-1,1]
            P.z = baryCoord.x * pts[0].z + baryCoord.y * pts[1].z + baryCoord.z * pts[2].z;
            //��Ϊ����-z����zԽ��--�������Խ��
            //x = idx % width��y = idx / width��idx = x + y * width
            if (zbuffer[int(P.x + P.y * width)] < P.z) {
                zbuffer[int(P.x + P.y * width)] = P.z;
                //���������ֵ
                Vec2i P_uv = (uv[0]) * baryCoord.x + (uv[1]) * baryCoord.y + (uv[2]) * baryCoord.z;
                //�������������ѯ�����൱��diffuse��k���Ҫ��intensity
                TGAColor color = model->diffuse(P_uv);
                image.set(P.x, P.y, TGAColor(color.bgra[2], color.bgra[1], color.bgra[0])*intensity);
            }
        }
    }
}

//����ӳ�� Gouraud shading�棬intensity��Ҫ���������ڲ���ֵ
void triangle(Vec3f* pts, float* zbuffer, TGAImage& image, Vec2i* uv, float* intensity) {
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);
    Vec2f bboxMax(0, 0);

    bboxMin.x = floor(std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMin.y = floor(std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y }));
    bboxMax.x = ceil(std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x }));
    bboxMax.y = ceil(std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y }));

    for (int i = bboxMin.x; i <= bboxMax.x; i++) {
        for (int j = bboxMin.y; j <= bboxMax.y; j++) {
            Vec3f P(i + 0.5, j + 0.5, 0.0); //�ж����������ǲ�������������
            Vec3f baryCoord = barycentric(pts, P);
            //����ϵ�����Ǹ�������������
            if (baryCoord.x < -0.01 || baryCoord.y < -0.01 || baryCoord.z < -0.01) {
                continue;
            }
            //��ʱ����������
            //z-buffer�жϸ��������ڵ������Ƿ�ɼ�
            //���������ֵ�õ����ص�P�����ֵz ��Χ[-1,1]
            P.z = baryCoord.x * pts[0].z + baryCoord.y * pts[1].z + baryCoord.z * pts[2].z;
            //��Ϊ����-z����zԽ��--�������Խ��
            //x = idx % width��y = idx / width��idx = x + y * width
            if (zbuffer[int(P.x + P.y * width)] < P.z) {
                zbuffer[int(P.x + P.y * width)] = P.z;
                //���������ֵ
                Vec2i P_uv = (uv[0]) * baryCoord.x + (uv[1]) * baryCoord.y + (uv[2]) * baryCoord.z;
                //�������������ѯ�����൱��diffuse��k���Ҫ��intensity
                TGAColor color = model->diffuse(P_uv);
                //intensity��ֵ
                float P_intensity = (intensity[0]) * baryCoord.x + (intensity[1]) * baryCoord.y + (intensity[2]) * baryCoord.z;
                //image.set(P.x, P.y, TGAColor(color.r * P_intensity, color.g * P_intensity, color.b * P_intensity, 255));
                image.set(P.x, P.y, TGAColor(color.bgra[2], color.bgra[1], color.bgra[0])* P_intensity);

            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
   // return Vec3f(int((v.x + 1.0) * width / 2 + 0.5), int((v.y + 1.0) * height / 2 + 0.5), v.z); //tinyrenderer
    return Vec3f((v.x + 1.0) * width / 2, (v.y + 1.0) * height / 2, v.z); //zw
}

int main(int argc, char** argv) {
    if (2 == argc) {
        model = new Model(argv[1]);
    }
    else {
        model = new Model("./african_head.obj");
    }

    float* zbuffer = new float[width * height];
    for (int i = 0; i < width * height; i++) {
        zbuffer[i] = -std::numeric_limits<float>::max();
    }

    TGAImage image(width, height, TGAImage::RGB);

    //ģ�;���ת���������ϵ��
    Matrix ModelView = lookat(eye, center, Vec3f(0, 1, 0));
    //ͶӰ����
    Matrix Projection = Matrix::identity(4);
    Matrix ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    Projection[3][2] = -1.0f / (eye-center).norm();

    std::cerr << ModelView << std::endl;
    std::cerr << Projection << std::endl;
    std::cerr << ViewPort << std::endl;
    Matrix z = (ViewPort * Projection * ModelView);
    std::cerr << z << std::endl;

    //Flat shading
    //for (int i = 0; i < model->nfaces(); i++) {
    //    std::vector<int> face = model->face(i);  //��������������      
    //    //Vec2i screenCoords[3]; //�����������������Ļ����
    //    Vec3f screenCoords[3]; //�����������������Ļ����
    //    Vec3f worldCoords[3];
    //    for (int j = 0; j < 3; j++) {
    //        worldCoords[j] = model->vert(face[j]); //��i����Ƭ�ĵ�j��������������꣬first�Ƕ������
    //        //screenCoords[j] = world2screen(worldCoords[j]); //��i����Ƭ�ĵ�j���������Ļ����,zû�� ����[-1,1]��xy�䵽[0,width]��[0,height]
    //        screenCoords[j] = Vec3f(ViewPort * Projection * v2m(worldCoords[j]));
    //    }
    //    //triangle(screenCoords, image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255));
    //    //������������Ƭ�ķ��߷��򡪡���Ϊɶ��ֱ����obj�ļ����vn��������ƽ������
    //    //����>��Ϊ���㷨�� = ���й���ö������ķ������ȡƽ��������һ����ķ��߲��ܼ򵥵����������㷨�����ȡƽ��
    //    //���㷨����������phong��gouraud shading
    //    Vec3f normal = (worldCoords[2] - worldCoords[0]) ^ (worldCoords[1] - worldCoords[0]);
    //    normal.normalize();
    //    float intensity = normal * light_dir; //���ߵ�˹�Դ����
    //    if (intensity > 0) {
    //        //�����޳���Χ�ڣ�������ӳ��
    //        Vec2i uv[3]; //�����������������������
    //        for (int j = 0; j < 3; j++) {
    //            uv[j] = model->uv(i, j); //��i��face����j������
    //        }
    //        //triangle(screenCoords, zbuffer, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
    //        triangle(screenCoords, zbuffer, image, uv, intensity);
    //    }
    //}

    //Gouraud shading
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);  //��������������      
        //Vec2i screenCoords[3]; //�����������������Ļ����
        Vec3f screenCoords[3]; //�����������������Ļ����
        Vec3f worldCoords[3];
        float intensity[3]; //intensity���飬sizeΪ3��ÿ��intensity��Ӧÿ���������ɫ
        Vec2i uv[3]; //uv���飬sizeΪ3��ÿ��uv��Ӧÿ�������uv����(��vec2i)
        for (int j = 0; j < 3; j++) {
            worldCoords[j] = model->vert(face[j]); //��i����Ƭ�ĵ�j��������������꣬first�Ƕ������
            //matrix->vector 4*1����������Ϊ3*1����������ʽ(Vec3f ���� ������geometry.cpp)
            //1.����obj�Ķ�����Ϊ�������ꡢ����λ���������w=1��2.���ModelView���䵽�������ϵ��
            //3.���Projection���γɽ���ԶС��Ч������Ϊ������Ľ���ԶС��Ч��������Ҫ�ȱ䵽�������ϵ����projection����
            //4.���ViewPort��ֹ�߽���ʧ
            screenCoords[j] = Vec3f(ViewPort * Projection * ModelView * v2m(worldCoords[j]));
            intensity[j] = std::max(model->norm(i, j) * light_dir, 0.f); //���ߵ�˹�Դ����
            uv[j] = model->uv(i, j);
        }
        //triangle(screenCoords, image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255));
        triangle(screenCoords, zbuffer, image, uv, intensity);
    }
    //�����ͼƬ
    image.flip_vertically(); // ���½���ԭ��
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}