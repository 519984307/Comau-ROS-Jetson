#pragma once
#include <QtCore/qbytearray.h>
#include <QFileDialog>
#include <QtCore/qsavefile.h>
#include <QtCore/qbuffer.h>
#include <QColorDialog>
#include <QtCore/qsettings.h>
#include <qapplication.h>
#include <qmainwindow.h>



class  ActionType
{
	
private:
	QSettings* settings;
public:
	ActionType(QSettings* settings) :settings(settings) {

	};
	~ActionType() {  };
	QByteArray addIcon()
	{
		printf("Insert icon Path: ");
		char stringa[10000];
		scanf("%s", stringa);
		bool enabled;
		QImage image(stringa);
		QByteArray arr;
		if (!image.isNull())
		{

			QBuffer bufferToSend(&arr);
			bufferToSend.open(QIODevice::WriteOnly);
			image.save(&bufferToSend, "jpeg");
		}
		settings->setValue("icon",stringa);
		return arr;
		/*QByteArray arr;
		QFileDialog dialog(d->w);
		dialog.setNameFilter(QSaveFile::tr("Images (*.png *.xpm *.jpg *.bmp)"));
		dialog.setViewMode(QFileDialog::Detail);
		dialog.setFileMode(QFileDialog::ExistingFile);

		if (QDialog::Accepted == dialog.exec())
		{

			QStringList filename = dialog.selectedFiles();
			QImage image(filename.at(0));
			if (!image.isNull())
			{

				QBuffer bufferToSend(&arr);
				bufferToSend.open(QIODevice::WriteOnly);
				image.save(&bufferToSend, "jpeg");
			}
			settings->setValue("icon", filename.at(0));
		}
		return arr;*/
	}
	QString addColor()
	{
		printf("Insert color: ");
		char stringa[10000];
		scanf("%s", stringa);

		QColor color(stringa);
		while (!color.isValid())
		{
			printf("Insert color: ");
			char stringa[10000];
			scanf("%s", stringa);
			 color=QColor(stringa);
		}
		settings->setValue("color", color.name(QColor::HexRgb));

		return color.name(QColor::HexRgb);
	}
	bool isEnabled()
	{
		printf("enable this type? y/n");
		char stringa[10];
		scanf("%s", stringa);
		bool enabled;
		if (QString(stringa).contains("y"))
			enabled= true;
		else
			enabled= false;
		settings->setValue("enabled", enabled);
		return enabled;
	}
	QString getName()
	{
		printf("Insert Action type name: ");
		char stringa[1000];
		scanf("%s", stringa);
		settings->setValue("typeName", stringa);
		return stringa;
	};
	QString getPreviousName()
	{
		return settings->value("typeName", "").toString();
	}

};

