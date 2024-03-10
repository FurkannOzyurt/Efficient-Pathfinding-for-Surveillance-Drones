public class Cordinate {
    public int row;
    public int col;

    public Cordinate(int y, int x){
        this.row = y;
        this.col = x;
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        Cordinate other = (Cordinate) obj;
        return col == other.col && row == other.row;
    }
}
