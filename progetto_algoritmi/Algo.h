#ifndef ALGO_H
#define ALGO_H

namespace Algo{
    // ======= Template classe vector<T> =======
  template <typename T>
  class vector {
  public:
      vector() : data(nullptr), m_size(0), m_capacity(0) {}

      vector(const vector<T>& other) {
          m_size = other.m_size;
          m_capacity = other.m_capacity;
          data = new T[m_capacity];
          for (uint64_t i = 0; i < m_size; i++) {
              data[i] = other.data[i];
          }
      }

      ~vector() {
          delete[] data;
      }

      void push_back(const T& elem) {
          if (m_size >= m_capacity) {
              reserve(m_capacity == 0 ? 1 : m_capacity * 2);
          }
          data[m_size++] = elem;
      }

      T& operator[](uint64_t index) {
          return data[index];
      }

      const T& operator[](uint64_t index) const {
          return data[index];
      }

      uint64_t size() const {
          return m_size;
      }
      
      void serial_print()
      {
        for(size_t i = 0; i < m_size;i++)
        {
          
          Serial.print(i);
          Serial.print(':');
          this->data[i].print();
          Serial.println(""); 
          
        }
        
        Serial.println("Fine!! :> ");
      } 

  private:
      T* data;
      uint64_t m_size;
      uint64_t m_capacity;

      void reserve(uint64_t new_capacity) {
          T* new_data = new T[new_capacity];
          for (uint64_t i = 0; i < m_size; i++) {
              new_data[i] = data[i];
          }
          delete[] data;
          data = new_data;
          m_capacity = new_capacity;
      }
  };

  // ======= Template classe pair<T,D> =======
  template <typename T, typename D>
  class pair {
  public:
      T first;
      D second;

      pair() : first(), second() {}
      pair(const T& a, const D& b) : first(a), second(b) {}

      void print()
      {
        Serial.print(this->first);
        Serial.print("-");
        Serial.print(this->second);
      }
  };

}

#endif // ALGO_H
