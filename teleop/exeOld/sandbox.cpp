std::vector<double> getVertexIndices(std::string const& pointLine)
{
  std::istringstream iss(pointLine);

  return std::vector<double>{ 
    std::istream_iterator<double>(iss),
    std::istream_iterator<double>()
  };
}