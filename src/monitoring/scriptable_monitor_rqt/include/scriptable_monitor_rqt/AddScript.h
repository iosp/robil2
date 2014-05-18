/********************************************************************************
** Form generated from reading UI file 'AddScript.ui'
**
** Created: Tue Dec 3 11:06:30 2013
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef ADDSCRIPT_H
#define ADDSCRIPT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextEdit>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_NewScriptForm
{
public:
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QPushButton *addButton;
    QSpacerItem *horizontalSpacer;
    QTextEdit *scriptText;

    void setupUi(QWidget *NewScriptForm)
    {
        if (NewScriptForm->objectName().isEmpty())
            NewScriptForm->setObjectName(QString::fromUtf8("NewScriptForm"));
        NewScriptForm->resize(431, 313);
        gridLayout_2 = new QGridLayout(NewScriptForm);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        addButton = new QPushButton(NewScriptForm);
        addButton->setObjectName(QString::fromUtf8("addButton"));

        gridLayout->addWidget(addButton, 1, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);

        scriptText = new QTextEdit(NewScriptForm);
        scriptText->setObjectName(QString::fromUtf8("scriptText"));

        gridLayout->addWidget(scriptText, 0, 0, 1, 2);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);


        retranslateUi(NewScriptForm);

        QMetaObject::connectSlotsByName(NewScriptForm);
    } // setupUi

    void retranslateUi(QWidget *NewScriptForm)
    {
        NewScriptForm->setWindowTitle(QApplication::translate("NewScriptForm", "Add new script", 0, QApplication::UnicodeUTF8));
        addButton->setText(QApplication::translate("NewScriptForm", "Add", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class NewScriptForm: public Ui_NewScriptForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // ADDSCRIPT_H
