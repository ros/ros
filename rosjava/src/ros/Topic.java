package ros;

public class Topic {

	public Topic(String name, String datatype, String md5sum) {
		this.name = name;
		this.datatype = datatype;
		this.md5sum = md5sum;
	}
	
	public String getName() {
		return name;
	}
	public String getDatatype() {
		return datatype;
	}
	public String getMd5sum() {
		return md5sum;
	}
	private String name;
	private String datatype;
	private String md5sum;
}
