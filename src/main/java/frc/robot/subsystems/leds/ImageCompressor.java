package frc.robot.subsystems.leds;

//Dr.Agomir Image compressor
import java.awt.*;
import java.io.*;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.Color;


public class ImageCompressor 
{
    public static BufferedImage loadImage(String path) // loads an image to the bufgfered image so it can be read
    {
        BufferedImage image = null; // creates new buffered image
        try
        {
            File file = new File(path); // turns the path into a readable file

            image = ImageIO.read(file); // reads that file

            if (image != null)
            {
            System.out.println(path + " Loaded!"); // shows that the image was loaded if it works
            }
            else
            {
            System.out.println("failed!"); // tells us it was failed
            }

        }
        catch (IOException e) // catches the exception
        {
            e.printStackTrace();
        }
        return image; // returns the image
    }
    public static int[][][] compressImage(BufferedImage image, int width, int height) // compresses it and returns a color 2D array
    {
        int[][][] compressedImage = new int[height][width][3]; // makes a new array
        for (double i = 0; i < image.getHeight(); i += image.getHeight()/(double) height)
        {
            for (double g = 0; g < image.getWidth(); g += image.getWidth()/(double) width)
            {
                int averageR = 0; // makes an empty average for this area of the image
                int averageG = 0; // makes an empty average for this area of the image
                int averageB = 0; // makes an empty average for this area of the image
                int pixlesCounter = 0;
                for (int i2 = 0; i2 < image.getHeight()/height; i2++)
                {
                    for (int g2 = 0; g2 < image.getWidth()/width; g2++)
                    {
                        try
                        {
                            Color pixel = new Color(image.getRGB(g2 + (int)g, i2 + (int)i));
                            pixlesCounter += 1;
                            averageR += pixel.getRed(); // adds the r value of the pixel to the average
                            averageG += pixel.getGreen(); // adds the g value of the pixel to the average
                            averageB += pixel.getBlue(); // adds the b value of the pixel to the average
                        }
                        catch(ArrayIndexOutOfBoundsException e)
                        {


                        }
                        if ((int) (g/(image.getWidth()/width)) == width - 1)//check if this is the last time g will run for this i
                        {
                            for (int g3 = ((int) g) + 1; g3 < image.getHeight(); g3++)//catch any extra pixles
                            {
                                try
                                {
                                    Color pixel = new Color(image.getRGB(g3, i2 + (int)i));
                                    pixlesCounter += 1;
                                    averageR += pixel.getRed(); // adds the r value of the pixel to the average
                                    averageG += pixel.getGreen(); // adds the g value of the pixel to the average
                                    averageB += pixel.getBlue();// adds the b value of the pixel to the average
                                }
                                catch(ArrayIndexOutOfBoundsException e)
                                {


                                }
                            }
                        }
                    }
                    if ((int) (i/(image.getHeight()/height)) == height - 1)//check if this is the last time i will run
                    {
                        for (int i3 = (int) i + 1; i3 < image.getHeight(); i3++)//catch any extra pixles
                        {
                            for (int g2 = 0; g2 < image.getWidth()/width; g2++)
                            {
                                    try
                                    {
                                        Color pixel = new Color(image.getRGB(g2 + (int)g, i3));
                                        pixlesCounter += 1;
                                        averageR += pixel.getRed(); // adds the r value of the pixel to the average
                                        averageG += pixel.getGreen(); // adds the g value of the pixel to the average
                                        averageB += pixel.getBlue(); // adds the b value of the pixel to the average
                                    }
                                    catch(ArrayIndexOutOfBoundsException e)
                                    {


                                    }
                                    if ((int) (g/(image.getWidth()/width)) == width - 1)//check if this is the last time g will run for this i
                                    {
                                        for (int g3 = ((int) g) + 1; g3 < image.getHeight(); g3++)//catch any extra pixles
                                        {
                                            try
                                            {
                                                Color pixel = new Color(image.getRGB(g3, i3));
                                                pixlesCounter += 1;
                                                averageR += pixel.getRed(); // adds the r value of the pixel to the average
                                                averageG += pixel.getGreen(); // adds the g value of the pixel to the average
                                                averageB += pixel.getBlue();// adds the b value of the pixel to the average
                                            }
                                            catch(ArrayIndexOutOfBoundsException e)
                                            {


                                            }
                                        }
                                    }
                            }
                        }
                    }
                }
                averageR = averageR/pixlesCounter; // finally calculates the average
                averageG = averageG/pixlesCounter; // finally calculates the average
                averageB = averageB/pixlesCounter; // finally calculates the average
                
                if (averageR > 254)
                {
                    averageR = 254;
                }
                if (averageG > 254)
                {
                    averageG = 254;
                }
                if (averageB > 254)
                {
                    averageB = 254;
                }
                if (averageR < 0)
                {
                    averageR = 0;
                }
                if (averageG < 0)
                {
                    averageG = 0;
                }
                if (averageB < 0)
                {
                    averageB = 0;
                }

                int i4 = (int) (i/(image.getHeight()/height)), g4 = (int) (g/(image.getWidth()/width));
                
                if (i4 >= height)
                {
                    i4 = height - 1;
                }
                if (g4 >= width)
                {
                    g4 = width - 1;
                }
                
                compressedImage[i4][g4][0] = averageR;//stes the "pixle" int the 2d array to the rgb values
                compressedImage[i4][g4][1] = averageG;//stes the "pixle" int the 2d array to the rgb values
                compressedImage[i4][g4][2] = averageB;//stes the "pixle" int the 2d array to the rgb values
                
                
            }


        }
    
        return compressedImage;//returns the array
    }

    public static BufferedImage[] loadVideoAsMultipleImages(String motherPath, int startingNum, int numberOfFiles)
    {
        BufferedImage[] videoRoll = new BufferedImage[numberOfFiles];
        for (int i = startingNum; i < videoRoll.length; i++)
        {
            String filePath = motherPath.substring(0, motherPath.indexOf("frame") + 5);
            int x = 0;
            if (i >= 10)
            {x++;}
            if (i >= 100)
            {x++;}
            if (i >= 1000)
            {x++;}
            if (i >= 10000)
            {x++;}
            for (int g = x; g < 4; g++)
            {
                filePath = filePath + 0;
            }
            System.out.println(filePath + i + motherPath.substring(motherPath.indexOf("frame") + 5));
            videoRoll[i] = loadImage(filePath + i + motherPath.substring(motherPath.indexOf("frame") + 5));
        }
        return videoRoll;
    }

    public static int[][][][] compressVideo(BufferedImage[] videoRoll, int width, int height)
    {
        int[][][][] compressedVideoRoll= new int[videoRoll.length][height][width][3];
        for (int i = 00000; i < videoRoll.length; i++)
        {
            compressedVideoRoll[i] = compressImage(videoRoll[i], width, height);
        }

        return compressedVideoRoll;
    }
    public static void main(String[] args)
    {
        
        //example1

        int[][][] testImageAsCompressed3dArrayComingToThearterMarch32_2111 = compressImage(loadImage("/Users/dragomirmateev/Downloads/compressor test image.png"), 4, 7);
        for (int i = 0; i < testImageAsCompressed3dArrayComingToThearterMarch32_2111.length; i++)
        {
            for (int f = 0; f < testImageAsCompressed3dArrayComingToThearterMarch32_2111[i].length; f++)
            {
                for (int p = 0; p < 3; p++)
                {
                    System.out.print(testImageAsCompressed3dArrayComingToThearterMarch32_2111[i][f][p] + ", ");
                }
                System.out.println();
            }
        }
        
        //example2

        int[][][][] testVideoAsCompressedArrayComingToTheatersMarch32_2111 = compressVideo(loadVideoAsMultipleImages("/Users/dragomirmateev/Downloads/test vid/frame.png", 0, 9),10,8);
        for (int i = 0; i < testVideoAsCompressedArrayComingToTheatersMarch32_2111.length; i++)
        {
            for (int g = 0; g < testVideoAsCompressedArrayComingToTheatersMarch32_2111[i].length; g++)
            {
                for (int f = 0; f < testVideoAsCompressedArrayComingToTheatersMarch32_2111[i][g].length; f++)
                {
                    System.out.print(testVideoAsCompressedArrayComingToTheatersMarch32_2111[i][g][f][0] + ",");
                    System.out.print(testVideoAsCompressedArrayComingToTheatersMarch32_2111[i][g][f][1] + ",");
                    System.out.println(testVideoAsCompressedArrayComingToTheatersMarch32_2111[i][g][f][2]);
                }
            }
        }
    }
}
