module ROS
  def ROS.parse_msg_path path
    raise ArgumentError, "bogus path" if not ARGV[0]
    p = path.split('/')
    pkg = p[p.length-3]
    name_no_ext = p[p.length-1].split('.')[0]
    {:pkg => pkg, :name_no_ext => name_no_ext}
  end
  def ROS.package_path pkg
    `rospack find #{pkg}`.chomp
  end
  def ROS.to_camelcase s
    s.gsub(/(^|_)(.)/) { $2.upcase }
  end
end

