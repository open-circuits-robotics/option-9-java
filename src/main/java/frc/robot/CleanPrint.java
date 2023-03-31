package frc.robot;

public class CleanPrint {
    private static int print_count;

    public CleanPrint()
    {
        print_count = 0;
    }

    public static void println(String s)
    {
        System.out.println(s);
        print_count++;
    }

    public static void reset()
    {
        while (print_count > 0)
        {
            System.out.print("\r");
            print_count--;
        }
    }


}
