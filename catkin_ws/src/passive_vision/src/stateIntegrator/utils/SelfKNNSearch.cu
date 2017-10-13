__global__ void SelfKNNSearch( int * result, const int * args, const float * pc1)
{
    
    int cudaNumBlocks = args[0];
    int cudaNumThreads = args[1];
    int K = args[2];
    int pc1NumPts = args[3];

    int pc1Idx = blockIdx.x * cudaNumThreads + threadIdx.x;

    // the point of reference
    
    float currPtX = pc1[pc1Idx * 3 + 0];
    float currPtY = pc1[pc1Idx * 3 + 1];
    float currPtZ = pc1[pc1Idx * 3 + 2];

    //float* nn_dists;  // a list of distances from low to high
    //int* nn_index;  // a list of indices corresponding to nn_dists
    float nn_dists[15];   
    int nn_index[15];
    
    float inf = 100000.0f;
    
    int w = 20;
    if (pc1Idx < pc1NumPts) {
      for(int i = 0; i < K; i++){
        nn_dists[i] = inf;
        nn_index[i] = pc1Idx;
      }
      
      //nn_dists = (float*)malloc(K * sizeof(float));
      //nn_index = (int*)malloc(K * sizeof(int));
      
      
      nn_dists[0] = 0.0;
      nn_index[0] = pc1Idx;
      
      //for(int dx = max(-w, 0); dx <= min(w, 480); dx++){
        //for(int dy = max(-w, 0); dy <= min(w, 640); dy++){
          
        for (int i = 0; i < pc1NumPts/* && i<60*/; i++) {
          
          float otherPtX = pc1[i * 3 + 0]; 
          float otherPtY = pc1[i * 3 + 1];
          float otherPtZ = pc1[i * 3 + 2];
          float dx = (currPtX - otherPtX), dy = (currPtY - otherPtY), dz = (currPtZ - otherPtZ);
          float dist = dx*dx + dy*dy + dz*dz;

          // find a place to insert
          int insert_idx = 0;
          if(i == pc1Idx){  // will be the first one
            continue;
          }
          else if(dist > 0.1*0.1 || dist > nn_dists[K-1]){  // dist too far or worse than the worst one in the current K list 
            // skip the point
            continue;
          }
          else{
            for(insert_idx = 0; insert_idx < K; insert_idx++){
              if(dist < nn_dists[insert_idx]){
                break;
              }
            }
          }
          
          // if found a place
          if(insert_idx < K){
            //~ // move the things after insert_idx back
            for(int j = K-1; j > insert_idx; j--){
              nn_dists[j] = nn_dists[j-1];
              nn_index[j] = nn_index[j-1];
            }
              
            //~ // save the result to the idx
            nn_dists[insert_idx] = dist;
            nn_index[insert_idx] = i;
          }
        }
      // copy the result
      
      for(int i = 0; i < K; i++){
        result[pc1Idx * K + i] = nn_index[i];  // later need to +1 get to matlab index  // the result will becomes K * pc1NumPts
      }
    }
}
