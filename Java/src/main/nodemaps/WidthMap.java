package main.nodemaps;

public interface WidthMap {
  void setSafeWidth(int w);

  void setActualWidth(int w);

  int getSafeWidth();

  int getActualWidth();
}
