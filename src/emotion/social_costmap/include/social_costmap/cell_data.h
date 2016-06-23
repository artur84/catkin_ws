#ifndef COSTMAP_CELL_DATA_H_
#define COSTMAP_CELL_DATA_H_
namespace social_costmap {
  /**
   * @class CellData
   * @brief Storage for cell information used during obstacle inflation
   */
  class CellData {
    public:
      /**
       * @brief  Constructor for a CellData object
       * @param  d The distance to the nearest obstacle, used for ordering in the priority queue
       * @param  i The index of the cell in the cost map
       * @param  x The x coordinate of the cell in the cost map
       * @param  y The y coordinate of the cell in the cost map
       * @param  sx The x coordinate of the closest obstacle cell in the costmap
       * @param  sy The y coordinate of the closest obstacle cell in the costmap
       * @return 
       */
      CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) : distance_(d), 
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {}
      double distance_;
      unsigned int index_;
      unsigned int x_, y_;
      unsigned int src_x_, src_y_;
  };

  /**
   * @brief Provide an ordering between CellData objects in the priority queue 
   * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b
   */
  inline bool operator<(const CellData &a, const CellData &b){
    return a.distance_ > b.distance_;
  }
};
#endif
