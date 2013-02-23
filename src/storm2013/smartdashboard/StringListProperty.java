/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package storm2013.smartdashboard;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.properties.PropertyHolder;
import javax.swing.table.TableCellRenderer;

/**
 *
 * @author Joe Grinstead
 */
public class StringListProperty extends Property {

    private String valueSplit = ", ";
    private String delimiter = ",";

    public StringListProperty(PropertyHolder parent, String name) {
        super(parent, name);
    }

    public StringListProperty(PropertyHolder parent, String name, String[] value) {
        super(parent, name, value);
    }

    protected void setValueSplit(String split) {
        valueSplit = split;
    }

    protected void setDelimeter(String delimiter) {
        this.delimiter = delimiter;
    }

    @Override
    protected String[] transformValue(Object value) {
        if (value instanceof String) {
            String text = (String) value;

            return text.split(delimiter);
        } else if (value instanceof String[]) {
            return (String[]) value;
        }
        return null;
    }

    @Override
    public Object getTableValue() {
        return getSaveValue();
    }

    @Override
    public TableCellRenderer getRenderer() {
        return null;
    }

    @Override
    public String getSaveValue() {
        String text = "";
        String[] value = getValue();
        for (int i = 0; i < value.length; i++) {
            if (i > 0) {
                text += valueSplit;
            }
            text += value[i];
        }
        return text;
    }

    public String[] getValue() {
        return (String[]) super.getValue();
    }
}
