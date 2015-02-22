#ifndef CLUSTERING_H
#define CLUSTERING_H

// Minimum ammount of similarity necessary for classes to be fused together
#define SIMILAR_THRESHOLD 0.3

#include <vector>

// Used for hierarquical clustering
struct Cluster {
  double com[3];             // Center of mass of the cluster
  double cbrt;               // Cube root of the volume estimate
  std::vector<int> indices;  // Indices of the regions of this cluster
};

class Clustering {
 public:
  // Perform hierarchical clustering on the cluster vector passed in
  static void hierarchicalClustering(std::vector<Cluster>& regions) {
    // While max similarity is larger than threshold and we have more than 1
    // region
    // Build 2D array of similarity
    // Find max similarity
    // If its less than threshold: Break
    // Else, fuse regions

    std::vector<std::vector<double> > table;

    while (regions.size() > 1) {
      updateSimilarityTable(regions, table);
      double similarity = updateRegions(regions, table);

      //The max similarity we found is below the threshold, stop clustering
      if (similarity == -1.0) {
        break;
      }
    }
  }

 private:
  // Only static methods, no need for explicit constructio
  Clustering();

  // Use the regions to build a similarity table
  static void updateSimilarityTable(std::vector<Cluster>& regions,
                                    std::vector<std::vector<double> >& table) {
    double distance, cbrt_average, similarity;

    int num_regions = regions.size();
    assert("Cluster vector has no regions!" && num_regions > 0);

    table.clear();

    for (int i = 0; i < num_regions; i++) {
      std::vector<double> column(i, 0);

      for (unsigned int j = 0; j < column.size(); j++) {
        distance = std::pow(regions[i].com[0] - regions[j].com[0], 2) +
                   std::pow(regions[i].com[1] - regions[j].com[1], 2) +
                   std::pow(regions[i].com[2] - regions[j].com[2], 2);
        distance = std::sqrt(distance);

        cbrt_average = (regions[i].cbrt + regions[j].cbrt) / 2.0;

        similarity = cbrt_average / distance;

        column[j] = similarity;
      }

      table.push_back(column);
    }
  }

  static double updateRegions(std::vector<Cluster>& regions,
                              std::vector<std::vector<double> >& table) {
    double max_similar = 0;
    int max_i = 0;
    int max_j = 0;

    for (unsigned int i = 0; i < table.size(); i++) {
      for (unsigned int j = 0; j < table[i].size(); j++) {
        if (table[i][j] > max_similar) {
          max_similar = table[i][j];
          max_i = i;
          max_j = j;
        }
      }
    }

    // If the largest similarity in the table is below the threshold, return
    // early
    if (max_similar < SIMILAR_THRESHOLD) return -1.0;

    Cluster& a = regions[max_i];
    Cluster& b = regions[max_j];

    int num_regions_a = a.indices.size();
    int num_regions_b = b.indices.size();

    // The cluster centers of mass are averages of all the centers of mass of
    // each region of the clusters. Before we fuse the two cluster COMs together
    // we need to reverse that averaging, by multiplying each with the number of
    // COMs that were used
    a.com[0] *= num_regions_a;
    a.com[1] *= num_regions_a;
    a.com[2] *= num_regions_a;
    b.com[0] *= num_regions_b;
    b.com[1] *= num_regions_b;
    b.com[2] *= num_regions_b;

    // Add the two COMs together
    a.com[0] += b.com[0];
    a.com[1] += b.com[1];
    a.com[2] += b.com[2];

    // Finalize the averaging of the COMs
    a.com[0] /= (num_regions_a + num_regions_b);
    a.com[1] /= (num_regions_a + num_regions_b);
    a.com[2] /= (num_regions_a + num_regions_b);

    // Add all the indices of 'b' to the end of 'a'
    a.indices.insert(a.indices.end(), b.indices.begin(), b.indices.end());

    // Weighted average of the two cbrts based on the number of regions
    a.cbrt = (a.cbrt * num_regions_a + b.cbrt * num_regions_b) /
             (num_regions_a + num_regions_b);

    // Remove cluster 'b' from the regions list: It has been fused into 'a'
    regions.erase(regions.begin() + max_j);

    return max_similar;
  }
};

#endif  // CLUSTERING_H
