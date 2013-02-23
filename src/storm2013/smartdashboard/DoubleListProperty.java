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
public class DoubleListProperty extends Property {

    private String valueSplit = ", ";
    private String delimiter = ",";

    public DoubleListProperty(PropertyHolder parent, String name) {
        super(parent, name);
    }

    public DoubleListProperty(PropertyHolder parent, String name, double[] value) {
        super(parent, name, value);
    }

    protected void setValueSplit(String split) {
        valueSplit = split;
    }

    protected void setDelimeter(String delimiter) {
        this.delimiter = delimiter;
    }

    @Override
    protected double[] transformValue(Object value) {
        if (value instanceof String) {
            String text = (String) value;

            String[] texts = text.split(delimiter);
            double[] values = new double[texts.length];

            for (int i = 0; i < texts.length; i++) {
                try {
                    values[i] = Double.parseDouble(texts[i].trim());
                } catch (NumberFormatException e) {
                    return null;
                }
            }
            return values;
        } else if (value instanceof double[]) {
            return (double[]) value;
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
        double[] value = getValue();
        for (int i = 0; i < value.length; i++) {
            if (i > 0) {
                text += valueSplit;
            }
            text += value[i];
        }
        return text;
    }

    public double[] getValue() {
        return (double[]) super.getValue();
    }
}
