float cx = 319.5; // center of projection
float cy = 239.5; // center of projection
float fx = 570.342224121093;//525.0;
float fy = 570.3422241210938;// 525.0;//            fx= cam_info_.K.at(0);
//fy= cam_info_.K.at(4);
//cx= cam_info_.K.at(2);
//cy= cam_info_.K.at(5);
std::cout << "cx:" << cx << " cy:" << cy <<" fx:" << fx << " fy" << fy << std::endl;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
cloud->header = pcl_conversions::toPCL(imgbuf_->imgPtr->header);
for(int r = 0; r < dimg->image.rows; r++){
   //Di = dimg->image.ptr<float>(r);
   Di = dimg->image.ptr<float>(r);
   Ii = rgbimg->image.ptr<uchar>(r);
   for(int c = 0; c < dimg->image.cols; c++){
       /*
       x = (1.0*(c - cx)) *Di[c]* fx;
       y = (1.0*(r - cy)) *Di[c]* fy;
       z = (1.0*Di[c]);
       */
       x = ((c - cx)*Di[c])/fx;
       y = (-1.0*(r - cy)*Di[c])/fy;
       z = (-1.0*Di[c]);
       if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)){
         if (m_save_ply)   {
               //fprintf(fOut, "%f %f %f ", x, y, z);
               //fprintf(fOut, "%f %f %f ", tran_x, tran_y, tran_z);
               //fprintf(fOut, "%d %d %d \n",
               //       (unsigned int)ptr[3*c], (unsigned int)ptr[3*c+1], (unsigned int)ptr[3*c+2]);
               //fprintf(fOut, "%d %d %f %f %f\n", r, c, x, y, z);
               fprintf(fOut, "%f %f %f %d %d %d \n", x, y, z,(unsigned int)Ii[3*c], (unsigned int)Ii[3*c+1], (unsigned int)Ii[3*c+2]);
           }
           copiedVerts++;
           pcl::PointXYZRGB p;
           //ALEN: Now going back into [m] for the visualiser
           //p.x = pt.x/1000.0;
           //p.y = pt.y/1000.0;
           //p.z = pt.z/1000.0;
           //ALEN: Now going back into [m] for the visualiser
           p.x = x;
           p.y = y;
           p.z = z;
           p.r = (unsigned int)Ii[3*c];
           p.g = (unsigned int)Ii[3*c+1];
           p.b = (unsigned int)Ii[3*c+2];
           cloud->points.push_back(p);
       }
   }
}
