#include <iostream>
#include <bits/stdc++.h>
#include <fstream>
#include "json.hpp"
#include <vector>
#include <bits/stdc++.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <cmath>
#include <eigen3/Eigen/Dense>

// Fill the zipped vector with pairs consisting of the
// corresponding elements of a and b. (This assumes 
// // that the vectors have equal length)
// template <typename A, typename B,typename C>
// void sample_trajectories(std::vector<std::vector<A>> &a, 
//      std::vector<std::vector<B>> &b, 
//      std::vector<C> &c,
//      std::vector<std::vector<A>>* ac, 
//      std::vector<std::vector<B>>* bc, 
//      std::vector<C>* cc, int file,int samples,float Observable, int index)
    
      
// {
//         if(c.size()<100)
//         { 
//           copy(a.begin(), a.end(), back_inserter(*ac));
          
//           copy(b.begin(), b.end(), back_inserter(*bc));
//           copy(c.begin(), c.end(), back_inserter(*cc));
//         }
//         else
//         { 
//           auto low_bound=std::lower_bound(c.begin(), c.end(),(Observable-0.005));
//           auto middle=std::lower_bound(c.begin(), c.end(),Observable);
//           auto upper_bound=std::lower_bound(c.begin(), c.end(),(Observable+0.005));
//           auto index_to_start_array=middle-c.begin();
//           auto index_to_end_array=c.end()-middle;
          
//           auto upper_index=upper_bound - c.begin();
//           auto lower_index=low_bound - c.begin();
//           auto middle_index=middle-c.begin();
          
//           if((upper_index-lower_index) >= 100 )
//           { if(upper_index-lower_index <= 1000 )
//           {
//               copy(&a[lower_index], &a[upper_index], back_inserter(*ac));
//               copy(&b[lower_index], &b[upper_index], back_inserter(*bc));
//               copy(&c[lower_index], &c[upper_index], back_inserter(*cc));
//           }
//           else
//           {   
//             if(index_to_start_array>=500 && index_to_end_array>=500)
//             {
//               auto samples_before=500;
//               auto samples_after=500;
//               copy(&a[middle_index-samples_before], &a[middle_index+samples_before], back_inserter(*ac));
//               copy(&b[middle_index-samples_before], &b[middle_index+samples_before], back_inserter(*bc));
//               copy(&c[middle_index-samples_before], &c[middle_index+samples_before], back_inserter(*cc));
//             }
//             else if (index_to_start_array<500)
//             {
//             auto samples_before=500+(500-index_to_end_array);
//             auto samples_after=500-(500-index_to_end_array);
//             copy(&a[middle_index-samples_before], &a[middle_index+samples_after], back_inserter(*ac));
//             copy(&b[middle_index-samples_before], &b[middle_index+samples_after], back_inserter(*bc));
//             copy(&c[middle_index-samples_before], &c[middle_index+samples_after], back_inserter(*cc));
//             }
//             else if(index_to_end_array<500)
//           { 
//             int samples_before=500-(500-index_to_start_array);
//             int samples_after=500+(500-index_to_start_array);
//             copy(&a[middle_index-samples_before], &a[middle_index+samples_after], back_inserter(*ac));
//             copy(&b[middle_index-samples_before], &b[middle_index+samples_after], back_inserter(*bc));
//             copy(&c[middle_index-samples_before], &c[middle_index+samples_after], back_inserter(*cc));
//             // std::cout<<"Inside Function"<<(*ac).size()<<std::endl;
//             // std::cout<<"Samples Before"<<samples_before<<std::endl;
//             // std::cout<<"Samples After"<<samples_after<<std::endl;
//           }
              
//           }
//           }
//           else
//           { if(index==2374)
//           {
//             std::cout<<"Inside Function0"<<std::endl;
//             std::cout<<"Start "<<index_to_start_array<<"End "<<index_to_end_array<<std::endl;
//           }
//             if(index_to_start_array>=50 && index_to_end_array>=50)
//             {
            
//             auto samples_before=50;
//             auto samples_after=50;
//             copy(&a[middle_index-samples_before], &a[middle_index+samples_after], back_inserter(*ac));
//             copy(&b[middle_index-samples_before], &b[middle_index+samples_after], back_inserter(*bc));
//             copy(&c[middle_index-samples_before], &c[middle_index+samples_after], back_inserter(*cc));
//             if(index==2374)
//           {
//             std::cout<<"Inside Function1"<<(*ac).size()<<std::endl;
//           }
//             }
//             else if(index_to_start_array<50)
//           { 
//             auto samples_before=50+(50-index_to_end_array);
//             auto samples_after=50-(50-index_to_end_array);
//             copy(&a[middle_index-samples_before], &a[middle_index+samples_after], back_inserter(*ac));
//             copy(&b[middle_index-samples_before], &b[middle_index+samples_after], back_inserter(*bc));
//             copy(&c[middle_index-samples_before], &c[middle_index+samples_after], back_inserter(*cc));
//             if(index==2374)
//           {
//             std::cout<<"Inside Function2"<<(*ac).size()<<std::endl;
//           }
//           }
//           else if(index_to_end_array<50)
//           { 
//             auto samples_before=50-(50-index_to_start_array);
//             auto samples_after=50+(50-index_to_start_array);
//             copy(&a[middle_index-samples_before], &a[middle_index+samples_after], back_inserter(*ac));
//             copy(&b[middle_index-samples_before], &b[middle_index+samples_after], back_inserter(*bc));
//             copy(&c[middle_index-samples_before], &c[middle_index+samples_after], back_inserter(*cc));
//             // std::cout<<"Inside Function"<<(*ac).size()<<std::endl;
//             // std::cout<<"Samples Before"<<samples_before<<std::endl;
//             // std::cout<<"Samples After"<<samples_after<<std::endl;
//             if(index==2374)
//           {
//             std::cout<<"Inside Function3"<<(*ac).size()<<std::endl;
//           }
//           }
//           }
//         if(index==2374 || index == 3466)
//           {
//             std::cout<<"Upper - Lower"<<(upper_index-lower_index)<<std::endl;
//             std::cout<<"Size"<<(*ac).size()<<std::endl;
//           }
          
//         }
        

// }
template <typename A, typename B,typename C>
void sample_trajectories(std::vector<std::vector<A>> &a, 
     std::vector<std::vector<B>> &b, 
     std::vector<C> &c,
     std::vector<std::vector<A>>* ac, 
     std::vector<std::vector<B>>* bc, 
     std::vector<C>* cc, int file,int samples,float Observable)
    
      
{
        
        if(c.size()<samples)
        { 
          copy(a.begin(), a.end(), back_inserter(*ac));
          
          copy(b.begin(), b.end(), back_inserter(*bc));
          copy(c.begin(), c.end(), back_inserter(*cc));
        }
        else
        { 
          auto low_bound=std::lower_bound(c.begin(), c.end(),Observable);
          auto index_to_start_x=low_bound-c.begin();
          auto index_to_end_x=c.end()-low_bound;
          auto index_to_start_y=low_bound-c.begin();
          auto index_to_end_y=c.end()-low_bound;
          
          if( index_to_end_x>=samples/2 && index_to_start_x>=samples/2)
          {
            // std::cout<<"FIRST IF CONDITION"<<std::endl;
            // std::cout<<"index_to_end_x>500 && index_to_start_x>500"<<index_to_end_x<<std::endl;
            copy(&a[index_to_start_x-samples/2], &a[index_to_start_x+samples/2], back_inserter(*ac));
            copy(&b[index_to_start_x-samples/2], &b[index_to_start_x+samples/2], back_inserter(*bc));
            copy(&c[index_to_start_x-samples/2], &c[index_to_start_x+samples/2], back_inserter(*cc));
            
          }
          else if(index_to_end_x<samples/2)
          { 
            // std::cout<<"index_to_end_x<500"<<std::endl;
            int samples_before=samples/2+(samples/2-index_to_end_x);
            int samples_after=samples/2-(samples/2-index_to_end_x);
            // std::cout<<"ERROR HERE"<<std::endl;
            copy(&a[index_to_start_x-samples_before], &a[index_to_start_x+samples_after], back_inserter(*ac));
            copy(&b[index_to_start_x-samples_before], &b[index_to_start_x+samples_after], back_inserter(*bc));
            copy(&c[index_to_start_x-samples_before], &c[index_to_start_x+samples_after], back_inserter(*cc));
            // std::cout<<"Inside Function"<<(*ac).size()<<std::endl;
            // std::cout<<"Samples Before"<<samples_before<<std::endl;
            // std::cout<<"Samples After"<<samples_after<<std::endl;
          }
          else if(index_to_start_x<samples/2)
          { 
            // std::cout<<"index_to_start_x<500"<<std::endl;
            int samples_before=samples/2-(samples/2-index_to_start_x);
            int samples_after=samples/2+(samples/2-index_to_start_x);
            copy(&a[index_to_start_x-samples_before], &a[index_to_start_x+samples_after], back_inserter(*ac));
            copy(&b[index_to_start_x-samples_before], &b[index_to_start_x+samples_after], back_inserter(*bc));
            copy(&c[index_to_start_x-samples_before], &c[index_to_start_x+samples_after], back_inserter(*cc));
            // std::cout<<"Inside Function"<<(*ac).size()<<std::endl;
            // std::cout<<"Samples Before"<<samples_before<<std::endl;
            // std::cout<<"Samples After"<<samples_after<<std::endl;
          }
        }
}
int data_to_index_helper( int i , float Observable_velocity,float Observable_velocity_x, float Observable_velocity_y,float Observable_pos_x, float Observable_pos_y,int partition_size)
{ 
  // std::cout<<" i is  " <<i<<  std::endl;
  if(i==0)
  {
    auto angle = atan2(Observable_velocity_y, Observable_velocity_x);
    angle += M_PI / partition_size;
    // auto vel_index=fmod(3.4,2.  );

    auto index= floor((fmod((angle + 2*M_PI) , (2*M_PI))) / (2*M_PI/(partition_size)));
    // std::cout<<"VELOCITY_INDEX " <<index<< std::endl;
    if(index==-1)
    { 
      index=0;
    }
    
    return index;
  }
  else if (i==1)
  {
    std::vector<int>  crosswalkx = {222 , 227};
    if(Observable_pos_x > crosswalkx[0] && Observable_pos_x < crosswalkx[1] )
    {
    // { std::cout<<"Crosswalk " << 0<< std::endl;
      return 0;
    }
    else if (Observable_pos_x < crosswalkx[0])
    {
      if(Observable_velocity_x > 0 && (crosswalkx[0] - Observable_pos_x ) <3)
      {
      // { std::cout<<"< 222 " <<1<< std::endl;
        return 1;
      }
    }
    else if (Observable_pos_x > crosswalkx[1])
    {
      if(Observable_velocity_x < 0 && ( Observable_pos_x - crosswalkx[1]) <3)
      {
      // {  std::cout<<">227 " <<1<< std::endl;
        return 1;
      }
    }
    // std::cout<<" Away from crosswalk " <<2<< std::endl;
    return 2;

  }
  else
  {
   if(Observable_pos_y < -3.5)
   {  
    //  std::cout<<" On footpath " << 1 << std::endl;
     return 1;
   }   
   else if (Observable_pos_y > 3.5)
   {
     return 2;
    //  std::cout<<" On footpath " << 2 << std::endl;
   }
  //  std::cout<<" On Road " <<0 << std::endl;
   return 0;
  }
}
int data_to_index(float Observable_velocity,float Observable_velocity_x, float Observable_velocity_y,float Observable_pos_x, float Observable_pos_y,int vel_partition_size )
{       
        
        std::vector<int>  partition_vector = {2 , 3 , 3};
        std::vector<int>  partition_stride (3);
        float partition_size=1;
        for(auto i=0;i<partition_vector.size();i++)
        {
          partition_size *= partition_vector[i];
          
        }
        auto temp_partition_size=partition_size;
        auto index = 0;
        for(auto i=0;i<partition_vector.size();i++)
        { 
          temp_partition_size /= partition_vector[i];
          partition_stride[i]= int(temp_partition_size);
          auto new_index=data_to_index_helper(i, Observable_velocity, Observable_velocity_y, Observable_velocity_x,  Observable_pos_x,  Observable_pos_y,  partition_vector[i]);
          index += int(new_index*temp_partition_size);
          // std::cout <<index<< std::endl;
           
        }


        // auto angle = atan2(Observable_velocity_y, Observable_velocity_x);
        // angle += M_PI / vel_partition_size;
        // // auto vel_index=fmod(3.4,2.  );
        // auto vel_index= floor((fmod((angle + 2*M_PI) , (2*M_PI))) / (2*M_PI/(vel_partition_size)));
        // index=vel_index;
        
        // index=vel_index;
        

                
        return index;

}
int main(int argc, char* argv[])
{  
  using namespace nlohmann;    
  std::vector<std::vector<float>>  v2_x2;
  std::vector<std::vector<float>> v2_y2;
  std::vector<std::vector<std::vector<float>>>  v3_x,v4_x;
  std::vector<std::vector<std::vector<float>>> v3_y,v4_y;
  std::vector<std::vector<float>> v3_o,v4_o;
  std::vector<std::vector<float>> ac;
  std::vector<std::vector<float>> bc;
  std::vector<float> cc;
  std::vector<int> itr;
  int window=40;
  int no_of_files;
  std::ifstream meta_read("/home/anish/Downloads/json_data/meta.txt");
  meta_read>>no_of_files;
  std::cout<<no_of_files<<std::endl;
  std::vector<std::vector<float>> v; 
  json j;
  for (int file=0;file<no_of_files;file++)
  {
    std::ifstream file_read("/home/anish/Downloads/json_data/json_data_"+std::to_string(file)+".json");
    file_read>>j;
    std::cout << "/home/anish/Downloads/json_data/json_data_"+std::to_string(file)+".json"<< std::endl;
    std::vector<float> v1;
    std::vector<float> v2_x(window);
    std::vector<float> v2_y(window);
    std::vector<std::vector<float>> v2_x1;
    std::vector<std::vector<float>>  v2_y1;
    std::vector<std::vector<float>>  v2(2);
    float* arr={};

    for(int i=0;i<j.size();i++)
    { 
      float observable_value=roundf(float(j[std::to_string(i)]["Observable"])*10000)/10000;
      
      v1.push_back(observable_value);
      for (int k=0;k<window;k++)
        {  
          std::string trajectory_numberx="x"+std::to_string(k);
          std::string trajectory_numbery="y"+std::to_string(k);
          v2_x[k]=(float(j[std::to_string(i)]["Trajectory_x"][trajectory_numberx]));
          v2_y[k]=(float(j[std::to_string(i)]["Trajectory_y"][trajectory_numbery]));

        }

      v2_x1.push_back(v2_x);
      v2_y1.push_back(v2_y);
      
    }
    // std::cout<<std::is_sorted(v1.begin(),v1.end())<<std::endl; 
    v3_o.push_back(v1);
    v3_x.push_back(v2_x1);
    v3_y.push_back(v2_y1);
      if(file==0)
      {
      v2_x2= v2_x1;
      v2_y2= v2_y1;     
      }
      else
      {
      v2_x2.insert( v2_x2.end(), v2_x1.begin(), v2_x1.end() );
      v2_y2.insert( v2_y2.end(), v2_y1.begin(), v2_y1.end() );
      }
      itr.push_back(v2_x1.size());


 }
// //  for (int file=0;file<no_of_files;file++)
// //  {
// //   sample_trajectories(v3_x[file],v3_y[file],v3_o[file],&ac, &bc, &cc,file,1000,2);    
// //   std::cout<<ac.size()<<std::endl;
// //     auto max=max_element(std::begin(cc), std::end(cc));
// //     std::cout<<"MAXIMUM"<<*max<<std::endl;
// //     auto min=min_element(std::begin(cc), std::end(cc));
// //     std::cout<<"MINIMUM"<<*min<<std::endl;
// //  }
// // int file=0;
   
    std::ifstream file_read("/home/anish/Downloads/json_data/test_data.json");
    file_read>>j;
    // for (int traj=0;traj<j.size();traj+=1)
    for (int traj=0;traj<10000;traj+=1)
    {
      float observable_vel=j[std::to_string(traj)]["Observable_Velocity"];
      // std::cout<<"VELOCITY:"<<observable_vel<<std::endl;
      float observable_vel_x=j[std::to_string(traj)]["Observable_Velocity_x"];
      // std::cout<<observable_vel_x<<std::endl;
      float observable_vel_y=j[std::to_string(traj)]["Observable_Velocity_y"];
      // std::cout<<observable_vel_y<<std::endl;
      float observable_pos_x=j[std::to_string(traj)]["Observable_Position_X"];
      // std::cout<<observable_pos_x<<std::endl;
      float observable_pos_y=j[std::to_string(traj)]["Observable_Position_Y"];
      // std::cout<<observable_pos_y<<std::endl;
      float angle = j[std::to_string(traj)]["Sidewalk_Angle"];
      auto index=j[std::to_string(traj)]["Index"];
      // std::cout<<"Index"<<index<<std::endl;
      auto trial_index= data_to_index(observable_vel,observable_vel_x, observable_vel_y,observable_pos_x, observable_pos_y,no_of_files );
      // std::cout<<"INDEX IS"<<index<<std::endl;
      // std::cout<<"TRIAL INDEX IS"<<trial_index<<std::endl;


      sample_trajectories(v3_x[index],v3_y[index],v3_o[index],&ac, &bc, &cc,index,1000,observable_vel);    
      // std::cout<<v3_x[index].size()<<std::endl;
      auto max=max_element(std::begin(cc), std::end(cc));
      // std::cout<<"MAXIMUM"<<*max<<std::endl;
      auto min=min_element(std::begin(cc), std::end(cc));
      // std::cout<<"MINIMUM"<<*min<<std::endl;
      typedef std::vector<std::vector<Eigen::VectorXd>> SampleVector;
      SampleVector real_samples;
      real_samples.resize(window);
      std::ofstream file_write("/home/anish/Downloads/json_data/output/output"+std::to_string(traj)+".txt");
      std::vector<float>  prev_pos_x(20);
      std::vector<float> prev_pos_y(20);
      float prev_k;
      float dt=0.1;
      
        for (int s=0;s<ac.size();s++)
        {   
          for(int i=0; i<20;i++)
        {
            prev_pos_x[i]=observable_pos_x;
            prev_pos_y[i]=observable_pos_y;
        }
        int prev_k=0;
        float temp_x = ac[s][0];
        float temp_y = bc[s][0]; 
        ac[s][0] = ac[s][0]*cos(angle) - bc[s][0]*sin(angle);
        bc[s][0] = ac[s][0]*sin(angle) + bc[s][0]*cos(angle);
        temp_x = ac[s][1];
        temp_y = bc[s][1];
        ac[s][1] = temp_x*cos(angle) - temp_y*sin(angle);
        bc[s][1] = temp_x*sin(angle) + temp_y*cos(angle);
        prev_pos_x[0]=ac[s][0]*dt + ac[s][1]*dt + observable_pos_x;
        prev_pos_y[0]=bc[s][0]*dt + bc[s][1]*dt + observable_pos_y;
        file_write<<prev_pos_x[0];
        file_write<<" ";
        file_write<<prev_pos_y[0  ];
        file_write<<"\n";
          for (int k=1;k<20;k++)
          { 
            prev_k=k-1;
            float temp_x = ac[s][2*k];
            float temp_y = bc[s][2*k]; 
            ac[s][2*k] = temp_x*cos(angle) - temp_y*sin(angle);
            bc[s][2*k] = temp_x*sin(angle) + temp_y*cos(angle);
            temp_x = ac[s][2*k+1];
            temp_y = bc[s][2*k+1];
            ac[s][2*k+1] = temp_x*cos(angle) - temp_y*sin(angle);
            bc[s][2*k+1] = temp_x*sin(angle) + temp_y*cos(angle);
            prev_pos_x[k]=ac[s][2*k]*dt + ac[s][2*k+1]*dt + prev_pos_x[prev_k];
            prev_pos_y[k]=bc[s][2*k]*dt + bc[s][2*k+1]*dt + prev_pos_y[prev_k];

            file_write<<prev_pos_x[k];
            file_write<<" ";
            file_write<<prev_pos_y[k];
            file_write<<"\n";
          // real_samples[k][0](s)=bc[s][k];
          
          }
          
        }
      file_write.close();
      ac.clear();
      bc.clear();
      cc.clear();
    }
  return 0;
}