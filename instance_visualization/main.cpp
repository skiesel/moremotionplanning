#include <fstream>
#include <thread>

#include "assimp_mesh_loader.hpp"
#include "opengl_wrapper.hpp"

int main(int argc, char *argv[]) {
	std::string filename(argv[1]);
	if(filename.find(".vert") != std::string::npos) {
		const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

		std::vector<double> point(28, 0);
		//0,1,2,3    position
		point[3] = 1;
		//4,5,6,7    normal
		point[6] = point[7] = 1;
		//8,9,10,11  color
		point[11] = 1;
		// 12-27 transform
		point[12+0] = point[12+5] = point[12+10] = point[12+15] = 1;

		unsigned int current = 1;
		bool first = true;
		std::chrono::milliseconds framerate(250);
		std::chrono::time_point<std::chrono::system_clock> start, end;
		auto lambda = [&]() {
			std::chrono::duration<double, std::milli> elapsedms = end-start;
			start = std::chrono::system_clock::now();
			glPointSize(2);

			fprintf(stderr, "current: %s\n", argv[current]);
			if(!first) {
				std::this_thread::sleep_for(framerate - elapsedms);
			} else {
				first = false;
			}
			std::fstream fs;
			fs.open(argv[current], std::fstream::in);

			std::vector<double> points;
			while(fs.good()) {
				fs >> point[0] >> point[1] >> point[2] >> point[8] >> point[9] >> point[10];
				points.insert(points.end(), point.begin(), point.end());
			}

			opengl.drawPoints(points);

			fs.close();
			current++;
			if(current >= argc) {
				current = 1;
			}

			 end = std::chrono::system_clock::now();
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, 10);
	} else if(filename.find(".edge") != std::string::npos) {
		const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();

		unsigned int current = 1;
		bool first = true;
		std::chrono::milliseconds ms(1000);

		double x0, y0, z0, x1, y1, z1, r, g, b;
		auto lambda = [&]() {
			glPointSize(2);

			fprintf(stderr, "current: %s\n", argv[current]);
			if(!first) {
				std::this_thread::sleep_for(ms);
			} else {
				first = false;
			}
			std::fstream fs;
			fs.open(argv[current], std::fstream::in);

			std::vector<double> points;
			while(fs.good()) {
				fs >> x0 >> y0 >> z0 >> x1 >> y1 >> z1 >> r >> g >> b;
				OpenGLWrapper::Color color(r, g, b);
				opengl.drawLine(x0, y0, z0, x1, y1, z1, color);
			}

			fs.close();
			current++;
			if(current >= argc) {
				current = 1;
			}
		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda, 10);
	} else if(filename.find(".dae") != std::string::npos) {
		AssimpMeshLoader environment(argv[1]);

		std::vector< std::vector<AssimpMeshLoader::Vertex> > vertices;
		std::vector< std::vector<AssimpMeshLoader::Triangle> > triangles;
		std::vector< std::vector<double> > normals;

		environment.get(vertices, triangles, normals);


		const OpenGLWrapper &opengl = OpenGLWrapper::getOpenGLWrapper();
		auto color = OpenGLWrapper::Color::Red();

		std::vector<double> transform(16, 0);
		transform[0] = transform[5] = transform[10] = transform[15] = 1;

		auto lambda = [&]() {

			for(unsigned int i = 0; i < triangles.size(); ++i) {
				const std::vector<AssimpMeshLoader::Vertex> &verts = vertices[i];
				const std::vector<AssimpMeshLoader::Triangle> &tris = triangles[i];
				std::vector<double> pts(84, 1.0);

				auto c = color.getColor();
				for(auto tri : tris) {
					unsigned int cur = 0;
					for(unsigned int i = 0; i < 3; ++i) {
						for(unsigned int j = 0; j < 3; ++j) {
							pts[cur++] = verts[tri[i]][j];
						}
						cur++; // add one extra for the 4th vector component
						for(unsigned int j = 0; j < 3; ++j) {
							pts[cur++] = normals[tri[i]][j];
						}
						cur++; // add one extra for the 4th vector component
						for(unsigned int j = 0; j < 4; ++j) {
							pts[cur++] = c[j];
						}
						for(unsigned int j = 0; j < 16; ++j) {
							pts[cur++] = transform[j];
						}
					}

					opengl.drawTriangles(pts);
					//opengl.drawPoints(pts);
				}
			}

		};
		OpenGLWrapper::getOpenGLWrapper().runWithCallback(lambda);
	} else {
		fprintf(stderr, "unrecognized file type\n");
	}

	return 0;
}